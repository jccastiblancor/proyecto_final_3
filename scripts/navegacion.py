#!/usr/bin/env python
import time
import rospy
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from proyecto_final_3.srv import Image

"""
metodos usados en la clase minheap 
"""


def compareTo(this, that):
    return this[0] >= that[0]


def parent(i):
    return int((i - 1) / 2)


"""
Clase que implementa una cola de prioridad.
Los datos ingresados son parejas de datos de la forma [prioridad, data]
"""


class MinHeap:

    # Constructor to initialize a heap
    def __init__(self):
        self.heap = []
        self.values = []

    # ordena el min heap
    def order(self, i):
        if not compareTo(self.heap[i], self.heap[parent(i)]):
            self.heap[i], self.heap[parent(i)] = self.heap[parent(i)], self.heap[i]
            self.order(parent(i))

    # Inserts a new key 'k'
    def insert(self, k, val):
        self.heap.append([k, val])
        self.order(len(self.heap) - 1)

    # Decrease value of key at index 'i' to new_val
    # It is assumed that new_val is smaller than heap[i]
    def decreaseKey(self, i, new_val):
        for j in range(len(self.heap)):
            if self.heap[j][1] == i:
                break
        self.heap[j] = [new_val, i]
        self.order(j)

    #  cambia la posicion de 2 nodos en el minheap
    def bajar(self, i, size):
        if i * 2 + 2 >= size:
            pass
        elif compareTo(self.heap[i * 2 + 1], self.heap[i * 2 + 2]):
            if not compareTo(self.heap[i * 2 + 2], self.heap[i]):
                self.heap[i], self.heap[i * 2 + 2] = self.heap[i * 2 + 2], self.heap[i]
                self.bajar(i * 2 + 2, len(self.heap))
        else:
            if not compareTo(self.heap[i * 2 + 1], self.heap[i]):
                self.heap[i], self.heap[i * 2 + 1] = self.heap[i * 2 + 1], self.heap[i]
                self.bajar(i * 2 + 1, len(self.heap))

    # Method to remove minium element from min heap
    def extractMin(self):
        self.heap[0], self.heap[len(self.heap) - 1] = self.heap[len(self.heap) - 1], self.heap[0]
        sacar = self.heap.pop()
        self.bajar(0, len(self.heap))
        return sacar

    # This functon deletes key at index i. It first reduces
    # value to minus infinite and then calls extractMin()
    def deleteKey(self, i):
        self.decreaseKey(i, float("-inf"))
        self.extractMin()

    # Get the minimum element from the heap
    def getMin(self):
        return self.heap[0]

    # Dice si el heap esta vacio
    def is_empty(self):
        return len(self.heap) == 0


def euclidiana(nodo, fin):
    """
    Calcula la distancia euclidiana entre 2 nodos del gridmap
    Parameters
    ----------
    nodo : [Integer, Integer]
        nodo en el gridmap
    fin : [Integer, Integer]
        Nodo final
    Return
    ----------
    ans: Integer
        distancia euclidiana
    """
    return ((nodo[0] - fin[0]) ** 2 + (nodo[1] - fin[1]) ** 2) ** (1 / 2)


def manhattan(nodo, fin):
    """
    Calcula la distancia de manhattan entre 2 nodos del gridmap
    Parameters
    ----------
    nodo : [Integer, Integer]
        nodo en el gridmap
    fin : [Integer, Integer]
        Nodo final
    Return
    ----------
    ans: Integer
        distancia de manhattan
    """
    ans = abs(nodo[0] - fin[0])
    ans += abs(nodo[1] - fin[1])
    return ans


def A(grafo, heuristica, inicio, final):
    """
    Implementa el algoritmo A*
    Parameters
    ----------
    grafo : dictonary
        grafo que representa el gridmap
    heuristica : String
        heuristica que va a usar el algoritmo puede ser Manhattan o euclidiana
    inicio : arr[Integer]
        Nodo de inicio
    final : arr[Integer]
        Nodo final
    Return
    ----------
    ans: Arr[arr[Integer]]
        ruta optima
    explorados: Arr[arr[Integer]]
        nodos explorados
    """
    explorados = []  # lista de nodos explorados
    anterior = []  # lista de parejas de datos de la forma [nodo_anterior, nodo_actual]
    priority_queue = MinHeap()  # cola de prioridad
    end = False  # Variable para indicar si se llego al nodo destino
    ans = []  # lista para guardar la ruta optima
    visitados = {}  # diccionario para indicar que nodos han sido visitados
    for nodo in grafo:  # poner todos los nodos como no visitados
        visitados[nodo] = False

    nodo = inicio
    visitados[nodo] = True
    priority_queue.insert(0, [inicio, 0])  # agregar el nodo de inicio a la cola de prioridad

    while not priority_queue.is_empty() and not end:  # recorrer hasta llegar al destino o hasta recorrer todos los nodos
        obj = priority_queue.extractMin()[1]  # saca el nodo con menor costo
        nodo = obj[0]
        cn_ant = obj[1]
        ad = grafo[nodo]
        explorados.append(nodo)  # agregar nodo a la lista de exploracion
        for i in ad:
            if not visitados[i[1]]:
                cn = cn_ant + 0.1  # Se asume costo de movimiento entre celdas de 0.1
                if heuristica == 'm':
                    costo = cn + manhattan(i[1], final)
                else:
                    costo = cn + euclidiana(i[1], final)
                priority_queue.insert(costo, [i[1], cn])  # agrega el nodo a la cola de prioridad
                visitados[i[1]] = True

                anterior.append([nodo, i[1]])
                if i[1] == final:  # si llega al nodo destino recorre la lista 'anterior' para retornar la ruta
                    end = True
                    next = nodo
                    ans.append(next)
                    for j in anterior:
                        if j[1] == nodo:
                            next = j[0]

                    while next != inicio:
                        ans.append(next)
                        for k in anterior:
                            if k[1] == next:
                                next = k[0]
    return ans, explorados


def dijkstra(grafo, inicio, destino):
    """
    Implementa el algoritmo de dijkstra
    Parameters
    ----------
    grafo : dictonary
        grafo que representa el gridmap
    inicio : arr[Integer]
        Nodo de inicio
    destino : arr[Integer]
        Nodo final
    Return
    ----------
    ans: Arr[arr[Integer]]
        ruta optima
    explorados: Arr[arr[Integer]]
        nodos explorados
    """
    ans = []  # variable en la que se va a guardar la ruta
    anterior = {}  # diccionario en el que se guardara la informacion del nodo anterior para construir
    end = False  # variable para definir si ya termino
    priority_queue = MinHeap()  # queue
    visitados = {}  # diccionario de nodso visitados
    dist = {}  # distancia desde el nodo de inicio hasta el nodo en la key del diccionario
    explorados = []  # nodos explorados
    for nodo in grafo:  # inicia los nodos con costo infinito y los asigna como no visitados
        visitados[nodo] = False
        priority_queue.insert(sys.maxsize, nodo)
        dist[nodo] = sys.maxsize

    nodo = inicio
    priority_queue.decreaseKey(nodo, 0)
    dist[nodo] = 0  # pone el primer nodo con costo 0 y lo agrega en la cola de prioridad

    while not priority_queue.is_empty() and not end:
        nodo = priority_queue.extractMin()[1]  # saca el nodo con menor costo
        explorados.append(nodo)  # agrega el nodo a la lista de exploracion
        if nodo == destino:  # si llega al destino recorre la lista 'anterior' para retornar la ruta
            end = True
            next = nodo
            while next != inicio:
                ans.append(next)
                next = anterior[next]
            ans.append(inicio)
        else:
            visitados[nodo] = True
            ad = grafo[nodo]
            for a in ad:  # recorre los nodos adyacentes
                costo = dist[nodo] + 0.1  # se asume costo de 0.1
                # Si el nodo no se ha visitado y el costo calculado es menor al que se tenia previamente
                if not visitados[a[1]] and costo < dist[a[1]]:
                    anterior[a[1]] = nodo
                    dist[a[1]] = costo
                    priority_queue.decreaseKey(a[1], costo)  # actualiza el costo en la cola de prioridad
    return ans, explorados


# este es un ejemplo de una funcion que genera un grafo a partir del gridmap_resized
# teniendo en cuenta la vecindad para cada celda, asumiendo que las acciones discretas del
# robot son cuatro: moverse al norte, al sur, al oriente y al este.
def gridmap2graph(gridmap, w, h):
    # cambiar el gridmap a valores 0 o 1
    maze = np.where(gridmap == 254, 0, gridmap)  # celda libre
    maze = np.where(gridmap == 0, 1, maze)  # celda ocupada
    # recorremos todas las celdas del maze para formar un diccionario, donde el key es la coordenada
    # (i,j) de la celda y sus valores son las coordenadas de los vecinos + la accion del robot que lleva
    # a cada uno
    graph = {(i, j): [] for j in range(w) for i in range(h) if not maze[i][j]}
    for fila, col in graph.keys():
        if fila > 0 and not maze[fila - 1][col]:
            graph[(fila, col)].append(("N", (fila - 1, col)))
        if fila > 0 and col < w and not maze[fila - 1][col + 1]:
            graph[(fila, col)].append(("NE", (fila - 1, col + 1)))
        if fila > 0 and col > 0 and not maze[fila - 1][col - 1]:
            graph[(fila, col)].append(("NW", (fila - 1, col - 1)))
        if col < w and not maze[fila][col + 1]:
            graph[(fila, col)].append(("E", (fila, col + 1)))
        if col > 0 and not maze[fila][col - 1]:
            graph[(fila, col)].append(("W", (fila, col - 1)))
        if fila < h and not maze[fila + 1][col]:
            graph[(fila, col)].append(("S", (fila + 1, col)))
        if fila < h and col > 0 and not maze[fila + 1][col - 1]:
            graph[(fila, col)].append(("SW", (fila + 1, col - 1)))
        if fila < h and col < w and not maze[fila + 1][col + 1]:
            graph[(fila, col)].append(("SE", (fila + 1, col + 1)))
    return graph


def get_graph(imagen):
    # leer el gridmap
    gridmap = cv2.imread(imagen, -1)
    # convertir celdas de las que no se tiene informacion a celdas ocupadas, ya que
    # ninguna de las dos se tendran en cuenta en la generacion del grafo
    gridmap[(gridmap >= 179) & (gridmap <= 238)] = 0
    gridmap[(gridmap >= 241) & (gridmap <= 255)] = 255

    # dilatar los obstaculos, tal que las celdas en blanco simbolicen aquellos espacios
    # libres por donde puede pasar el centro del robot. Asi se garantiza que no se acerque
    # a los obstaculos
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    gridmap_dilatated = cv2.dilate(cv2.bitwise_not(gridmap), kernel, iterations=1)
    gridmap_dilatated = cv2.bitwise_not(gridmap_dilatated)

    # la resolucion original de la imagen es 1px=5m
    # reescalamos la imagen para que 1px=20cm
    scale_percent = 25  # percent of original size
    width = int(gridmap_dilatated.shape[1] * scale_percent / 100)
    height = int(gridmap_dilatated.shape[0] * scale_percent / 100)
    dim = (width, height)
    gridmap_resized = cv2.resize(gridmap_dilatated, dim, interpolation=cv2.INTER_NEAREST)
    # cv2.imwrite('gridmap_resized.pgm', gridmap_resized)

    # plt.imshow(gridmap_resized, cmap='gray', vmin=0, vmax=255)
    return gridmap2graph(gridmap_resized, width, height), gridmap_resized


def navegacion(grafo, img, metodo, inicio, final, heuristica):
    """
    ejecuta el algoritmo recibido por parametro para la navegacion del robot.
    ----------
    grafo : dictonary
        grafo que mapea la informaci'on del entorno del robot
    img: matrix [Integer, Integer]
        imagen del gridmap
    metodo : string
        algoritmo de navegacion
    inicio : arr[Integer]
        Nodo de inicio
    final : arr[Integer]
        Nodo final
    hueristica : string
        heuristica para el algoritmo A*
    """

    rospy.wait_for_service('add_two_ints')
    try:
        servicio = rospy.ServiceProxy('Image', Image)
        resp1 = servicio()
        return resp1.points
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    rospy.init_node('navegacion', anonymous=True)
    if metodo == 'A':
        ans, explorados = A(grafo, heuristica, inicio, final)
    elif metodo == 'd':
        ans, explorados = dijkstra(grafo, inicio, final)

    backtorgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)  # pasa la imagen a RGB
    backtorgb[inicio[0], inicio[1]] = (0, 255, 0)  # pinta el inicio de color verde
    backtorgb[final[0], final[1]] = (255, 0, 0)  # pinta el final de color rojo
    plt.show(block=False)

    for k in range(len(explorados)):  # Pinta los nodos recorridos de color azul
        if explorados[k] == inicio:
            backtorgb[explorados[k][0], explorados[k][1]] = (0, 255, 0)
        elif explorados[k] == final:
            backtorgb[explorados[k][0], explorados[k][1]] = (255, 0, 0)
        else:
            backtorgb[explorados[k][0], explorados[k][1]] = (40, 112, 255)
            plt.imshow(backtorgb)

            plt.pause(0.001)

    for a in range(len(ans)):  # pinta los nodos del camnino respuesta de color naranja
        if ans[a] == inicio:
            backtorgb[ans[len(ans) - 1 - a][0], ans[len(ans) - 1 - a][1]] = (0, 255, 0)
        elif ans[a] == final:
            backtorgb[ans[len(ans) - 1 - a][0], ans[len(ans) - 1 - a][1]] = (0, 255, 0)
        else:
            backtorgb[ans[len(ans) - 1 - a][0], ans[len(ans) - 1 - a][1]] = (255, 112, 40)
            print(ans[len(ans) - 1 - a])
            plt.imshow(backtorgb)
            plt.show(block=False)

            plt.pause(0.05)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()




if __name__ == '__main__':
    ruta_imagen = './src/taller5_3/scripts/results/map_pioneer.pgm'
    args = rospy.myargv(argv=sys.argv)  # argumentos por consola. c:
    if len(args) == 3:
        h = args[2]
        metodo = args[1]
    elif len(args) == 2:
        metodo = args[1]
        h = 'm'
    else:
        h = 'm'
        metodo = 'A'

    grafo, imagen = get_graph(ruta_imagen)
    inicio = (29, 29)  # nodo inicio
    final = (11, 11)  # nodo final

    try:
        navegacion(grafo, imagen, metodo, inicio, final, h)
    except rospy.ROSInterruptException:
        pass
