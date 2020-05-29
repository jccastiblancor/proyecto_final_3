"""
Clase que implementa una cola de prioridad.
Los datos ingresados son parejas de datos de la forma [prioridad, data]
"""
import sys


def compareTo(this, that):
    return this[0] >= that[0]


def parent(i):
    return int((i - 1) / 2)


class MinHeap:

    # Constructor to initialize a heap
    def __init__(self):
        self.heap = []
        self.values = []

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
        self.heap[i] = new_val
        self.order(i)

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

    def is_empty(self):
        return len(self.heap) == 0


def adyacentes(visitados, nodo):
    a = []
    x = nodo[0]
    y = nodo[1]
    if x != 0:
        if not visitados[x - 1][y]:
            nuevo = [x - 1, y]
            a.append(nuevo)
    if y != 0:
        if not visitados[x][y - 1]:
            nuevo = [x, y - 1]
            a.append(nuevo)
    if x != len(visitados) - 1:
        if not visitados[x + 1][y]:
            nuevo = [x + 1, y]
            a.append(nuevo)
    if y != len(visitados[0]) - 1:
        if not visitados[x][y + 1]:
            nuevo = [x, y + 1]
            a.append(nuevo)
    return a


def manhattan(nodo, fin):
    ans = abs(nodo[0] - fin[0])
    ans += abs(nodo[1] - fin[1])
    return ans / 10


def A(visitados, matriz, inicio, fin):
    anterior = []
    nodo = inicio
    priority_queue = MinHeap()
    end = False
    ans = []
    ad = adyacentes(visitados, nodo)
    visitados[nodo[0]][nodo[1]] = True
    for i in ad:
        cn = matriz[i[0]][i[1]]
        costo = cn + manhattan([i[0], i[1]], fin)
        priority_queue.insert(costo, [i, cn])
        visitados[i[0]][i[1]] = True
        anterior.append([nodo, i])

    while not priority_queue.is_empty() and not end:
        print(priority_queue.heap)
        obj = priority_queue.extractMin()[1]
        nodo = obj[0]
        cn_ant = obj[1]
        ad = adyacentes(visitados, nodo)
        for i in ad:
            cn = cn_ant + matriz[i[0]][i[1]]
            costo = cn + manhattan([i[0], i[1]], fin)
            priority_queue.insert(costo, [i, cn])
            visitados[i[0]][i[1]] = True
            anterior.append([nodo, i])
            if i == fin:
                end = True
                ans.append(nodo)
                for i in anterior:
                    if i[1] == nodo:
                        next = i[0]
                while next != inicio:
                    ans.append(next)
                    for i in anterior:
                        if i[1] == next:
                            next = i[0]
        visitados[nodo[0]][nodo[1]] = True
    return ans

# Funtion that implements Dijkstra's single source
# shortest path algorithm for a graph represented
# using adjacency matrix representation
def dijkstra(src):

    dist = [sys.maxint] * V
    dist[src] = 0
    sptSet = [False] * V

    for cout in range(V):

        # Pick the minimum distance vertex from
        # the set of vertices not yet processed.
        # u is always equal to src in first iteration
        u = self.minDistance(dist, sptSet)

        # Put the minimum distance vertex in the
        # shotest path tree
        sptSet[u] = True

        # Update dist value of the adjacent vertices
        # of the picked vertex only if the current
        # distance is greater than new distance and
        # the vertex in not in the shotest path tree
        for v in range(self.V):
            if self.graph[u][v] > 0 and sptSet[v] == False and \
                    dist[v] > dist[u] + self.graph[u][v]:
                dist[v] = dist[u] + self.graph[u][v]

    self.printSolution(dist)




def encontrar_mejor_ruta(matriz, inicio, fin):
    visitados = []
    for i in range(len(matriz)):
        visitados.append([])
        for j in range(len(matriz[0])):
            if matriz[i][j] >= 0.5:
                visitados[i].append(True)
            else:
                visitados[i].append(False)
    ans = []
    ans = A(visitados, matriz, inicio, fin)
    return ans


data_path = 'matriz.in'
f = open(data_path, 'r')
lines = f.readlines()
m = []
for i in range(len(lines)):
    a = lines[i]
    a = a.split()
    if i == 0:
        y_ini = int(a[0])
        x_ini = int(a[1])
        y_end = int(a[2])
        x_end = int(a[3])
    else:
        m.append([])
        for j in a:
            m[i - 1].append(float(j))

nodo_inicio = [x_ini, y_ini]
nodo_end = [x_end, y_end]

lista = encontrar_mejor_ruta(m, nodo_inicio, nodo_end)
d = {0: 'A', 1: 'B', 2: 'C', 3: 'D', 4: 'E'}
respuesta = []
for i in lista:
    respuesta.append(d[i[0]] + str(i[1] + 1))
print(respuesta)
