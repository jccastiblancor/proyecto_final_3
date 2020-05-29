# Taller 4

## Problema

La figura 1 1a muestra el gridmap del entorno de un robot diferencial y 
la figura 1b la probabilidad para cada celda de que esté ocupada. El robot 
tiene discretizadas 4 posibles acciones: moverse hacia el norte,
sur, este u oeste.

![robotica0](https://user-images.githubusercontent.com/53923936/79271581-a861cd80-7e65-11ea-867f-7573227cb9fe.PNG)

![robotica1](https://user-images.githubusercontent.com/53923936/79271608-b31c6280-7e65-11ea-91e9-bef196913848.PNG)

Para este ejercicio se ha establecido que no es posible realizar transición 
hacia una celda que tiene p ≥ 0:5.

El objetivo consiste en encontrar la ruta que debe recorrer el robot para ir 
desde S hasta G. Para ello, usted solucionará los siguientes algoritmos de 
búsqueda: DFS, BFS y A*. Para la solución, tenga en cuenta:

* El orden para añadir nodos a la cola siempre será el siguiente: primero se 
añade el nodo al norte, luego al oeste luego al sur y por último, al este.
* Como heurística usará distancia de Manhattan entre el estado actual y el 
estado destino G dividida en 10. Por ejemplo: h(S) = 6=10 y h(b4) = 5=10
* El costo de la transición de una celda x a una celda y corresponderá a la 
probabilidad de ocupación de la celda destino.
* El grafo de búsqueda se irá formando a medida que se van explorando nodos. 
Nunca se debe expandir un nodo que ya haya sido visitado.