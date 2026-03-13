
#include "WiFiS3.h"


char ssid[] = "ROBOT_WIFI";
char pwd[] = "ros_robot";

IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(80);

struct clients
{
  IPAddress ip;
  unsigned long lastConnection;
  bool active = false;  
};

const int MAX_CLIENTS = 10;
clients listClients[MAX_CLIENTS];


void setup() {
  
  Serial.begin(9600);

  WiFi.config(local_IP, gateway, subnet);

  Serial.print("Inicializando punto de acceso... SSID: ");
  Serial.println(ssid);

  if (WiFi.beginAP(ssid, pwd, 11) != WL_AP_LISTENING)
  {
    Serial.println("Error: No se pudo crear la red");
    while(true);
  }

  server.begin();
  
  Serial.print("Red lista! IP del Arduino: ");
  Serial.println(WiFi.localIP());

}

void loop() {

  WiFiClient client = server.available();

  if (client) {
    IPAddress ipActual = client.remoteIP();
    int device = -1;

    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (listClients[i].active && listClients[i].ip == ipActual) {
        device = i;
        break;
      }
    }

    if (device == -1) {
      for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!listClients[i].active) {
          listClients[i].ip = ipActual;
          listClients[i].active = true;
          device = i;
          Serial.print("[INFO] Nuevo dispositivo con IP ");
          Serial.print(ipActual);
          Serial.println(" conectado!");
          break;
        }
      }
    }

    if (device != -1) {
      listClients[device].lastConnection = millis();
    }

    client.stop();
  }

  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (listClients[i].active && (millis() - listClients[i].lastConnection > 15000)) {
      Serial.print("[INFO] El dispositivo con la IP ");
      Serial.print(listClients[i].ip);
      Serial.println(" ha dejado la red.");
      listClients[i].active = false;
    }
  }

  delay(2000);
  
}
