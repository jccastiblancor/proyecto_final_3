import cv2
import numpy as np
import matplotlib.pyplot as plt

# leer el gridmap
gridmap = cv2.imread('./mapa/map_pioneer.pgm',-1)
# convertir celdas de las que no se tiene información a celdas ocupadas, ya que
# ninguna de las dos se tendrán en cuenta en la generacion del grafo
gridmap = np.where(gridmap==205, 0, gridmap)

# dilatar los obstaculos, tal que las celdas en blanco simbolicen aquellos espacios
# libres por donde puede pasar el centro del robot. Asi se garantiza que no se acerque
# a los obstaculos
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
gridmap_dilatated = cv2.dilate( cv2.bitwise_not(gridmap), kernel, iterations=1)
gridmap_dilatated = cv2.bitwise_not(gridmap_dilatated)

# la resolucion original de la imagen es 1px=5m
# reescalamos la imagen para que 1px=20cm
scale_percent = 25 # percent of original size
width = int(gridmap_dilatated.shape[1] * scale_percent / 100)
height = int(gridmap_dilatated.shape[0] * scale_percent / 100)
dim = (width, height)
gridmap_resized = cv2.resize(gridmap_dilatated, dim, interpolation = cv2.INTER_NEAREST)
# cv2.imwrite('gridmap_resized.pgm', gridmap_resized)
plt.imshow(gridmap_resized, cmap='gray', vmin=0, vmax=255)
plt.show()

# este es un ejemplo de una funcion que genera un grafo a partir del gridmap_resized
# teniendo en cuenta la vecindad para cada celda, asumiendo que las acciones discretas del
# robot son cuatro: moverse al norte, al sur, al oriente y al este.
def gridmap2graph(gridmap):
    # cambiar el gridmap a valores 0 o 1
    maze = np.where(gridmap==254, 0, gridmap) #celda libre
    maze = np.where(gridmap==0, 1, maze) #celda ocupada
    # recorremos todas las celdas del maze para formar un diccionario, donde el key es la coordenada
    # (i,j) de la celda y sus valores son las coordenadas de los vecinos + la accion del robot que lleva
    # a cada uno
    graph = {(i, j): [] for j in range(width) for i in range(height) if not maze[i][j]}
    for row, col in graph.keys():
        if row < height - 1 and not maze[row + 1][col]:
            graph[(row, col)].append(("S", (row + 1, col)))
            graph[(row + 1, col)].append(("N", (row, col)))
        if col < width - 1 and not maze[row][col + 1]:
            graph[(row, col)].append(("E", (row, col + 1)))
            graph[(row, col + 1)].append(("W", (row, col)))
    return graph


# main
graph = gridmap2graph(gridmap_resized)
print(graph)