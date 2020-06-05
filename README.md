proyecto_final_3
# Dependencias

``
sudo apt install python-tornado python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx
``


# Para correr el proyecto

Correr roscore y los nodos necesarios para establecer la conexion con la UI 
con el comando: 

```
roslaunch rosbridge_server rosbridge_websocket.launch
pip3 install numpy
```

Tambien debe de iniciar vrep y cargar la escena. En caso de tener un ambiente
virtual para el proyecto este se debe de activar antes de correr los nodos.

Para abrir la UI abra el archivo index.html en la carpeta src del proyecto.

Luego abra cuatro terminales en las que debe correr los nodos del proyecto 
con los siguientes comandos:

```
rosrun proyecto_final_3 navegacion_server.py 
rosrun proyecto_final_3 gridmap_points_server.py
rosrun proyecto_final_3 vision.py
rosrun proyecto_final_3 traccion.py
```

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