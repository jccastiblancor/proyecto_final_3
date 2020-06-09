#!/usr/bin/env python
import sys

import numpy as np
import rospy
import cv2
from proyecto_final_3.srv import Dibujo, DibujoResponse, GridmapPoints
from PIL import Image, ImageDraw


class Ruta:

    def __init__(self):
        self.ruta_imagen = './src/proyecto_final_3/data/map_scene2.pgm'  # ruta de la imagen
        self.ruta_resultados = './src/proyecto_final_3/results/'
        self.ruta = []  # ruta optima
        self.inicio = None  # nodo inicio
        self.grafo = None  # grafo de exploracion

    def jirafa(self, derecha, izquierda, up, down, nodo):
        if (derecha + izquierda) > (up + down):  # Es ancho el espacio de dibujo
            for i in range(up - 5):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Diagonal derecha por lomo
                self.ruta.append(nodo)
            for i in range(up - 6):
                nodo = self.dar_nodo_direccion('N', nodo)  # Arriba por la cola
                self.ruta.append(nodo)
            for i in range(up - 1):
                nodo = self.dar_nodo_direccion('E', nodo)  # Recto a la derecha hasta la pata
                self.ruta.append(nodo)
            for i in range(up - 4):
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Diagonal izquierda para pata 1
                self.ruta.append(nodo)
            for i in range(up - 4):
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Izquierda hasta llegar a la pata
                self.ruta.append(nodo)
            for i in range((up - 1) * 2):
                nodo = self.dar_nodo_direccion('W', nodo)  # Izquiera por el cuello hasta la cabeza
                self.ruta.append(nodo)
            for i in range(up - 6):
                nodo = self.dar_nodo_direccion('S', nodo)  # Abajo por la boca
                self.ruta.append(nodo)
            for i in range(up - 6):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Diagonal izquierda arriba hasta la cabeza
                self.ruta.append(nodo)
            for i in range(up - 6):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Diagonal arriba derecha hasta cuello
                self.ruta.append(nodo)
            for i in range(up - 4):
                nodo = self.dar_nodo_direccion('E', nodo)  # Derecha hasta finalizar
                self.ruta.append(nodo)
        else:  # es mas largo que ancho
            pass

    def gato(self, derecha, izquierda, up, down, nodo):
        if (derecha + izquierda) > (up + down):  # Es ancho el espacio de dibujo
            for i in range(6):
                nodo = self.dar_nodo_direccion('W', nodo)  # Se mueve del centro para arriba
                self.ruta.append(nodo)
            for i in range(3):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Se mueve diagonal arriba a la punta de la cabeza
                self.ruta.append(nodo)
            for i in range(21):
                nodo = self.dar_nodo_direccion('E', nodo)  # Se mueve diagonal abajo a la derecha hasta la punta del pico
                self.ruta.append(nodo)
            for i in range(6):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Se mueve a la izquierda hasa el cuello
                self.ruta.append(nodo)
            for i in range(3):
                nodo = self.dar_nodo_direccion('E', nodo)  # Diagonal derecha abajo para pecho
                self.ruta.append(nodo)
            for i in range(6):
                nodo = self.dar_nodo_direccion('S', nodo)  # Recto hacia abajo para pecho
                nodo = self.dar_nodo_direccion('E', nodo)
                self.ruta.append(nodo)
            for i in range(6):
                nodo = self.dar_nodo_direccion('S', nodo)
                self.ruta.append(nodo)
            for i in range(12):
                nodo = self.dar_nodo_direccion('W', nodo)  # Izquierda hasta llegar a la pata
                self.ruta.append(nodo)
            for i in range(3):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Diagonal abajo derecha hasta casi el piso
                self.ruta.append(nodo)
            for i in range(3):
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Diagonal abajo derecha hasta casi el piso
                self.ruta.append(nodo)
            for i in range(6):
                nodo = self.dar_nodo_direccion('W', nodo)
                self.ruta.append(nodo)
            for i in range(3):
                nodo = self.dar_nodo_direccion('E',nodo)  # Izquierda hasta llegar a la parte inferior izquierda del pato.
                nodo = self.dar_nodo_direccion('N', nodo)
                self.ruta.append(nodo)
        else:  # es mas largo que ancho
            pass
        # TODO pintar jirafa.

    def pato(self, derecha, izquierda, up, down, nodo):
        """
        Dibuja un pato
        """
        print('PATO')
        if (derecha + izquierda) > (up + down):  # Es ancho el espacio de dibujo
            for i in range(up - 4):
                nodo = self.dar_nodo_direccion('N', nodo)  # Se mueve del centro para arriba
                self.ruta.append(nodo)
            for i in range(up - 7):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Se mueve diagonal arriba a la punta de la cabeza
                self.ruta.append(nodo)
            for i in range(up - 5):
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('E',
                                               nodo)  # Se mueve diagonal abajo a la derecha hasta la punta del pico
                self.ruta.append(nodo)
            for i in range(up - 6):
                nodo = self.dar_nodo_direccion('W', nodo)  # Se mueve a la izquierda hasa el cuello
                self.ruta.append(nodo)
            for i in range(up - 8):
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Diagonal derecha abajo para pecho
                self.ruta.append(nodo)
            for i in range(up - 6):
                nodo = self.dar_nodo_direccion('S', nodo)  # Recto hacia abajo para pecho
                self.ruta.append(nodo)
            for i in range(up - 7):
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Diagonal izquierda para pecho
                self.ruta.append(nodo)
            for i in range(up - 6):
                nodo = self.dar_nodo_direccion('W', nodo)  # Izquierda hasta llegar a la pata
                self.ruta.append(nodo)
            for i in range(up - 7):
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Diagonal abajo derecha hasta casi el piso
                self.ruta.append(nodo)
            for i in range(up - 5):
                nodo = self.dar_nodo_direccion('W', nodo)  # Izquierda en la pata
                self.ruta.append(nodo)
            for i in range(up - 7):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Diagonal derecha arriba para la pata
                self.ruta.append(nodo)
            for i in range(up - 5):
                nodo = self.dar_nodo_direccion('W',
                                               nodo)  # Izquierda hasta llegar a la parte inferior izquierda del pato.
                self.ruta.append(nodo)
            for i in range(up - 8):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)  # Diagonal arriba derecha
                self.ruta.append(nodo)
            for i in range(up - 5):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)  # Diagonal izquierda arriba hasta la cola
                self.ruta.append(nodo)
            for i in range(up - 1):
                nodo = self.dar_nodo_direccion('E', nodo)  # Derecha todo el lomo
                self.ruta.append(nodo)
        else:  # es mas largo que ancho
            pass

    def pez(self, derecha, izquierda, up, down, nodo):
        """
        Dibuja un pez
        """

        if (derecha + izquierda) > (up + down):  # Es ancho el espacio de dibujo
            for i in range(up - 1):  # Se mueve para arriba hasta casi tocar la pared
                nodo = self.dar_nodo_direccion('N', nodo)
                self.ruta.append(nodo)
            for i in range(up - 3):  # dibuja la cara moviendose en diagonal, abajo a la derecha
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)
                self.ruta.append(nodo)
            nodo = self.dar_nodo_direccion('W', nodo)  # pequenio paso a la izquierda
            self.ruta.append(nodo)
            for i in range(up - 7):  # dibuja el cuerpo moviendose en diagonal abajo a la izquierda
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)
                self.ruta.append(nodo)
            for i in range(up - 6):  # dibuja la aleta de atras moviendose en diagonal abajo a la derecha
                nodo = self.dar_nodo_direccion('S', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)
                self.ruta.append(nodo)
            for i in range((up - 3) * 2):  # dibuja la aleta de atras moviendose a la izquierda (ya que el pez es
                # simetrico se multipllico la distancia por 2)
                nodo = self.dar_nodo_direccion('W', nodo)
                self.ruta.append(nodo)

            for i in range(up - 6):  # Pinta la otra mitad del pez
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)
                self.ruta.append(nodo)
            for i in range(up - 7):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('W', nodo)
                self.ruta.append(nodo)
            nodo = self.dar_nodo_direccion('W', nodo)
            self.ruta.append(nodo)
            for i in range(up - 3):
                nodo = self.dar_nodo_direccion('N', nodo)
                nodo = self.dar_nodo_direccion('E', nodo)
                self.ruta.append(nodo)
        else:  # es mas largo que ancho
            pass

    def gridmap2graph(self, gridmap, w, h):
        """
        Genera un grafo a partir del gridmap_resized teniendo en cuenta la vecindad para cada celda, asumiendo que las
        acciones discretas del robot son cuatro: moverse al norte, al sur, al oriente y al este.
        El grafo sera un diccionario donde el key es la coordenada (i,j) de la celda y sus valores son las coordenadas
        de los vecinos + la accion del robot que lleva a cada uno
        Parameters
        ----------
        gridmap : Integer[Integer[]]
            Imagen que representa el gridmap con el preprocesamiento aplicado
        w : Integer
            Ancho del gridmap
        h : Alto
            Alto del gridmap
        """

        maze = np.where(gridmap == 255, 0, gridmap)  # celda libre
        maze = np.where(gridmap == 0, 1, maze)  # celda ocupada

        graph = {(i, j): [] for j in range(w) for i in range(h) if not maze[i][j]}
        for row, col in graph.keys():
            if row < h - 1 and not maze[row + 1][col]:
                graph[(row, col)].append(("S", (row + 1, col)))
                graph[(row + 1, col)].append(("N", (row, col)))
            if col < w - 1 and not maze[row][col + 1]:
                graph[(row, col)].append(("E", (row, col + 1)))
                graph[(row, col + 1)].append(("W", (row, col)))
        self.grafo = graph

    def four_point_transform(self, image, pts):
        """
        realiza la transformada de la imagen a partir de las 4 esquinas seleccionadas
        """
        rect = []
        for j in range(4):
            rect.append([pts[j * 2], pts[j * 2 + 1]])

        rect = np.array(rect, dtype="float32")
        (tl, tr, br, bl) = rect
        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))
        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))
        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype="float32")
        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(rect, dst)
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        # return the warped image
        return warped

    def get_graph(self, points):
        """
        Obtiene un grafo a partir del gridmap proporcionado, aplicando un preprocesamiento al gridmap.
        Preprocesamiento de la forma:
        1. Leer gridmap y escalar el gridmap con los puntos seleccionados en el servicio gridmap_points
        2. convertir celdas de las que no se tiene informacion a celdas ocupadas
        3. dilatar los obstaculos
        4. Reescalar la imagen
        Return
        ----------
        gridmap_resized: Integer[Integer[]]
            Imagen del gridmap con el preprocesamiento aplicado
        """

        gridmap = cv2.imread(self.ruta_imagen, -1)

        gridmap = self.four_point_transform(gridmap, points)

        gridmap[(gridmap >= 179) & (gridmap <= 238)] = 0
        gridmap[(gridmap >= 241) & (gridmap <= 255)] = 255

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        gridmap_dilatated = cv2.dilate(cv2.bitwise_not(gridmap), kernel, iterations=1)
        gridmap_dilatated = cv2.bitwise_not(gridmap_dilatated)

        scale_percent = 25  # percent of original size
        width = int(gridmap_dilatated.shape[1] * scale_percent / 100)
        height = int(gridmap_dilatated.shape[0] * scale_percent / 100)
        dim = (width, height)
        gridmap_resized = cv2.resize(gridmap_dilatated, dim, interpolation=cv2.INTER_NEAREST)

        self.gridmap2graph(gridmap_resized, width, height)

        return gridmap_resized, width, height

    def save_ruta(self, gridmap):

        dim = (500, 500)
        backtorgb = cv2.cvtColor(gridmap, cv2.COLOR_GRAY2RGB)  # pasa la imagen a RGB

        for h in range(len(self.ruta)):  # pinta los nodos del camino respuesta de color naranja
            backtorgb[self.ruta[h][0], self.ruta[h][1]] = (255, 112, 40)

        backtorgb = cv2.resize(backtorgb, dim)
        cv2.imwrite(self.ruta_resultados + 'dibujo.png', backtorgb)

    def vrep_to_gridmap(self, start, ancho, alto):

        relacionX = float(15.0 / alto)
        relacionY = float(15.0 / ancho)

        rospy.loginfo('Nodo inicio vrep "{}"'.format(start))

        rospy.loginfo('Parametros metodo ancho:"{}"'.format(ancho))
        rospy.loginfo('Parametros metodo alto:"{}"'.format(alto))

        self.inicio = (int((start[0] + 7.5) / relacionX), int((start[1] + 7.5) / relacionY))
        rospy.loginfo('Nodo inicio "{}"'.format(self.inicio))

    def gridmap_to_vrep(self, alto, ancho):

        relacionX = 15.0 / alto
        relacionY = 15.0 / ancho

        direccion = None
        x = []
        y = []

        for nodo in self.ruta:
            x.append(nodo[0] * relacionX - 7.5)
            y.append(nodo[1] * relacionY - 7.5)
        print(x)
        print(y)
        return x, y

    def dar_nodo_direccion(self, direccion, nodo):
        adyacentes = self.grafo[nodo]
        for a in adyacentes:
            if a[0] == direccion:
                nodo = a[1]
                break
        return nodo

    def medir_distancia(self, direccion, nodo):

        contador = 0
        while nodo != None:
            adyacentes = self.grafo[nodo]
            for a in adyacentes:
                if a[0] == direccion:
                    nodo = a[1]
                    contador += 1
                    break
                nodo = None
        return contador

    def center(self):
        nodo = self.inicio
        self.ruta.append(nodo)

        up = self.medir_distancia('N', nodo)
        down = self.medir_distancia('S', nodo)
        derecha = self.medir_distancia('E', nodo)
        izquierda = self.medir_distancia('W', nodo)

        while abs(derecha - izquierda) > 1 or abs(up - down) > 1:
            up = self.medir_distancia('N', nodo)
            down = self.medir_distancia('S', nodo)
            derecha = self.medir_distancia('E', nodo)
            izquierda = self.medir_distancia('W', nodo)

            while abs(derecha - izquierda) > 1:
                derecha = self.medir_distancia('E', nodo)
                izquierda = self.medir_distancia('W', nodo)
                if derecha > izquierda:
                    nodo = self.dar_nodo_direccion('E', nodo)
                    self.ruta.append(nodo)
                else:
                    nodo = self.dar_nodo_direccion('W', nodo)
                    self.ruta.append(nodo)

            if up > down:
                nodo = self.dar_nodo_direccion('N', nodo)
                self.ruta.append(nodo)
            else:
                nodo = self.dar_nodo_direccion('S', nodo)
                self.ruta.append(nodo)

        return derecha, izquierda, up, down, nodo

    def dibujo(self, req):
        """
        Determina la ruta para el dibujo.
        ----------
        req : request
            estructura del servicio ed ROS
        """

        gridmap_points = rospy.ServiceProxy('gridmap_points', GridmapPoints)

        resp = gridmap_points()
        rospy.loginfo('Corriendo servicio gridmap points"{}"'.format(resp.points))

        rospy.loginfo('Corriendo servicio dibujo')
        gridmap_preprocesado, w, h = self.get_graph(resp.points)

        self.vrep_to_gridmap(req.inicio, w, h)

        self.ruta = []

        derecha, izquierda, up, down, nodo = self.center()
        if req.figura == 'pato':
            self.pato(derecha, izquierda, up, down, nodo)
        elif req.figura == 'pez':
            self.pez(derecha, izquierda, up, down, nodo)
        elif req.figura == 'jirafa':
            self.jirafa(derecha, izquierda, up, down, nodo)
        elif req.figura == 'gato':
            self.gato(derecha, izquierda, up, down, nodo)
        else:
            rospy.logerr('Metodo desconocido "{}"'.format(req.metodo))
            return None

        self.save_ruta(gridmap_preprocesado)

        response = DibujoResponse()
        response.rutax, response.rutay = self.gridmap_to_vrep(w, h)
        return response


def main():
    """
    main del servicio navegacion
    """
    rospy.init_node('dibujo_server', anonymous=True)

    ruta = Ruta()
    s = rospy.Service('dibujo', Dibujo, ruta.dibujo)
    print('========= Waiting for service ========')
    rospy.spin()


if __name__ == '__main__':
    main()
