# proyecto_final_3
## Dependencias

``
sudo apt install python-tornado python-pip ros-kinetic-rosbridge-suite ros-kinetic-web-video-server nginx
``


## Para correr el proyecto

* Correr roscore y los nodos necesarios para establecer la conexion con la UI 
con el comando: 

```
roslaunch rosbridge_server rosbridge_websocket.launch
```

* Luego debe de iniciar vrep y cargar la escena.
* en caso de tener un ambiente virtual para el proyecto este se debe de 
activar antes de correr los nodos de traccion y navegacion.
* Ahora se necesitan 5 nuevas terminales para correr los nodos del proyecto,
 realize el siguiente comando en las 5 terminales
 ```
source devel/setup.bash
catkin build
```

* Para abrir la UI abra el archivo index.html en la carpeta src del proyecto.

* Finalment para correr los nodos ejecute:

```
rosrun proyecto_final_3 dibujo_server.py
rosrun proyecto_final_3 navegacion_server.py 
rosrun proyecto_final_3 gridmap_points_server.py
rosrun proyecto_final_3 vision.py
rosrun proyecto_final_3 traccion.py
```

## Nodos

Se cuentan con los siguientes nodos
* traccion
* vision
* dibujo_server
* navegacion_server
* gridmap_points_server

## Servicios
Se tienen los siguientes servicios
* Navegacion
* Dibujo
* GridmapPoints

## Comunicacion

Se muestra el rafo generado con el comando rqt graph de la arquitectura
diseñada

![graph](https://user-images.githubusercontent.com/53923936/84152488-546d0280-aa2a-11ea-93ea-04ddeecff3a3.png)