proyecto_final_3
# Dependencias

``
sudo apt install python-tornado python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx
``


# Para correr el proyecto

Correr roscore con el comando 

``
roslaunch rosbridge_server rosbridge_websocket.launch
``

# Nodos

Se cuentan con los siguientes nodos
* Traccion
* Vision

# Servicios
Se tienen los siguientes servicios
* Navegacion
* Dibujar
* Imagen

# Topicos
Se van a usar para la comunicacion entre nodos los siguientes topicos
* ruta
*  

# Mensajes
Se cuentan con los siguientes tipos de mensaje
* ruta
* 