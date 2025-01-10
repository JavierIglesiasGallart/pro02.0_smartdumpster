#!/bin/bash

sudo apt update

cd $HOME
my_path=$(cat ~/.config/user-dirs.dirs | grep XDG_DOWNLOAD_DIR)
value_pos=$(expr index "$my_path" "=")
downloads_path=$(echo ${my_path:value_pos:${#my_path}})
downloads_path_clean=$(echo $downloads_path | sed 's/"//g')
goto_download_path="cd $downloads_path_clean"

#Instalación del Paquete de ROS Humble:
echo --- Instalando ROS Humble... ---
sleep 1s

	#Introducir instalación ROS2 Humble

echo --- Instalación de ROS Noetic completada! ---
sleep 1s

#Instalación del Servicio SSH:
echo --- Instalando el Servicio SSH... ---
sleep 1s

sudo apt install openssh-server

echo --- Instalación de SSH completada! ---
sleep 1s

#Instalación de las librerias Phidget (Control de los Motores):
echo --- Instalando las librerias de Phidget... ---
sleep 1s

sudo apt-get install libusb-1.0-0-dev
eval $goto_download_path

wget https://cdn.phidgets.com/downloads/phidget22/libraries/linux/libphidget22.tar.gz
tar -xf libphidget22.tar.gz
cd libphidget22-1.19.20240411
./configure
make
sudo make install

eval $goto_download_path

wget https://cdn.phidgets.com/downloads/phidget22/libraries/linux/libphidget22extra.tar.gz
tar -xf libphidget22extra.tar.gz
cd libphidget22extra-1.19.20240411
./configure
make
sudo make install

echo --- Instalación de librerias Phidget completada! ---
sleep 1s

#Instalación Phidget UDev Rules
echo --- Creando fichero de UDev Rules para Phidget... ---
sleep 1s

cd /$(find ~ -iname Ficheros_Instalación_SmartDumpster -type d)
sudo cp 99-libphidget22.rules ~/etc/udev/rules.d
cd /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger
cd ~

echo --- Creacicón de fichero de UDev Rules para Phidget completada! ---
sleep 1s

#Configuración conexión a red de ROSCore >> SmartDumpster
echo --- Configurando ROSCore para conectar a SmartDumpster... ---
sleep 1s

echo "#Configuración conexión a red de ROSCore >> SmartDumpster:" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.0.104:11311" >> ~/.bashrc
echo "export ROS_IP=$(hostname -I)" >> ~/.bashrc

echo --- Configuración de ROSCore completada! ---
sleep 1s

#Instalación ROSDep para conexión con mando PS4
echo --- Instalando ROSDep para conexión con mando PS4... ---
sleep 1s

sudo apt install ros-noetic-joy

echo --- Instalación de ROSDep completada! ---
sleep 1s

#Cierre
echo ---------------------------------------------
echo ---                                       ---
echo --- Configuración del entorno completada! ---
echo ---                                       ---
echo ---------------------------------------------
sleep 5s
