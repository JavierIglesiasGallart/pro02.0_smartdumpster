#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Configurar el ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42
echo "(+) ROS_DOMAIN_ID configurado en: $ROS_DOMAIN_ID"

# Hacer source del setup de ROS 2 (Jazzy en tu caso)
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "(+) Entorno ROS 2 Jazzy cargado."
else
    echo "(-) Error: No se encontró /opt/ros/jazzy/setup.bash"
fi

# Hacer source del setup 
if [ -f "$SCRIPT_DIR/smartdumpster2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/smartdumpster2_ws/install/setup.bash"
    echo "(+) Setup del proyecto cargado desde: smartdumpster2_ws/install/setup.bash"
else
    echo "(!) Aviso: No se encontró 'install/setup.bash'. ¿Has compilado ya con colcon?"
fi
