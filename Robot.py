#!/usr/bin/python
# -*- coding: UTF-8 -*-
# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''

# import brickpi3 # import the BrickPi3 drivers

import time     # import the time library for the sleep function
import sys
import random  # Simular perturbaciones
import numpy as np
import math
import cv2
from MapLib import Map2D
# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

# Posicion a polares segun se indica en las transparencias


def posicion_a_polar(x):
    dx = x[0]
    dy = x[1]
    theta = x[2]
    p = np.sqrt(dx * dx + dy * dy)
    beta = norm_pi(math.atan2(dy, dx) + np.pi)
    alpha = beta - theta
    return np.array([p, alpha, beta])


def loc(T, degrees=False):
    T = np.array(T)
    nx = T[0, 0]
    ny = T[1, 0]
    if degrees:
        nx = math.radians(nx)
        ny = math.radians(ny)
    theta = math.atan2(ny, nx)
    px = T[0, 2]
    py = T[1, 2]
    return np.array([px, py, theta])


# Funcion hom que crea la matriz T a partir de una posicion
def hom(x, degrees=False):
    x = np.array(x)
    dx = x[0]
    dy = x[1]
    theta = x[2]

    if degrees:
        theta = math.radians(theta)
    sin = math.sin(theta)
    cos = math.cos(theta)
    m = [[cos, -sin, dx], [sin, cos, dy], [0, 0, 1]]
    return np.array(m)


def norm_pi(th):
    th_norm = th
    while th_norm > np.pi:
        th_norm -= 2 * np.pi

    while th_norm < -np.pi:
        th_norm += 2 * np.pi

    return th_norm

# Dadas dos celdas, celda1 y celda2 devuelve la arista por la cual
# esta conectada la celda1 a la celda2
#   0
# 6   2
#   4


def arista(celda1, celda2):
    celda1 = np.array(celda1)
    celda2 = np.array(celda2)
    dif = celda2 - celda1

    if list(dif) == [0, 1]:
        return 0
    elif list(dif) == [1, 0]:
        return 2
    elif list(dif) == [0, -1]:
        return 4
    elif list(dif) == [-1, 0]:
        return 6
    else:
        return -1


class Robot:
    def __init__(self,  init_position=[0.0, 0.0, 0.0], log_file="log.txt"):
        """
            Clase para representar al robot
            x,y,th -> posicion x,y del robot y su orientacion
            v,w -> velocidades lineal y angular actuales
            V,W -> Listas con el registro de velocidades linear angular recogidas por readOdometry
            V_acc,W_acc -> Listas con el registro de velocidades establecidas con setSpeed
            POS -> Lista de las posiciones del robot
            error -> (Unused) Flag para indicar que se ha producido un error en la lectura de la velocidad
            tiempo_real -> indica si la simulacion se va a realizar en tiempo real
        """
        self.log = log_file
        self.f = None
        # odometry shared memory values
        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])

        # Velocidades lineal y angular actuales
        self.v = Value('d', 0)
        self.w = Value('d', 0)

        # Listas de log
        self.V = []
        self.W = []
        self.V_acc = []
        self.W_acc = []
        self.POS = [init_position]
        self.error = False

    """
        Prepara el fichero de log
    """

    def init_odometria(self):
        self.f = open(self.log, 'w')

    """
        Cierra el fichero de log
    """

    def stop_odometria(self):
        self.f.close()

    """
        Escribe en el fichero de log
    """

    def write_log(self, texto):
        self.f.write(texto+'\n')
    """

        Establece v como velocidad lineal del robot  y
        w como velocidad angular del robot
    """

    def setSpeed(self, v, w):

        self.v = Value('d', v)
        self.w = Value('d', w)

    """
        Rota en su posicion hasta alcanzar el angulo theta en T segundos
    """

    def rota(self, theta, T):
        pos = self.readOdometry()
        theta = norm_pi(theta)

        theta_R = norm_pi(pos[2])
        dif = theta - theta_R
        w = dif/T

        while abs(pos[2] - theta) > 0.02:
            pos = self.simubot([0, w], pos, .1)
            # print(math.degrees(pos[2]), math.degrees(theta))

            self.POS.append(pos)

        self.write_log("Rotando hasta angulo "+str(pos[2]))
        self.th = Value('d', pos[2])

    """
        Encuentra el angulo de rotacion inicial para llegar al objetivo con
        un Radio R
        El angulo lo encuentra incrementando por inc en cada iteración(radianes)
    """

    def encuentra_angulo(self, wXg, R, inc=.001):
        wXr = self.readOdometry()

        wTr = hom(wXr)
        wTg = hom(wXg)
        rTw = np.linalg.inv(wTr)
        rTg = rTw.dot(wTg)
        rXg = loc(rTg)
        x = rXg[0]
        y = rXg[1]
        R_est = (np.power(x, 2)+np.power(y, 2))/(2*y)
        theta = wXr[2]
        signo = -(R/abs(R))  # Para que no "De la vuelta" en sentido contrario
        while abs(R_est-R) > abs(R*0.01):  # Error del 1% (En radianes)
            # print(R_est)
            theta = norm_pi(theta + inc * signo)
            wTr = hom([wXr[0], wXr[1], theta])
            wTg = hom(wXg)
            rTw = np.linalg.inv(wTr)
            rTg = rTw.dot(wTg)
            rXg = loc(rTg)
            x = rXg[0]
            y = rXg[1]
            R_est = (np.power(x, 2)+np.power(y, 2))/(2*y)
        self.rota(theta, 0.2)
        return R_est

    """
        Alcanza el objetivo wXg con un error <= error en T segundos

        si R != describe una trayectoria con ese radio

        si rotar = True el robot rota en su posicion al inicio de la trayectoria de forma que sea una
        trayectoria realizable

        Cada 0.1 segundos simula la posicion del robot actualizando
        POS, V, W, V_acc, W_acc
        Además se actualiza la posicion del robot

        Implementación en bucle cerrado siguiendo un control PI

    """

    def alcanza_objetivo(self, wXg, error, R, T, rotar=True):
        wXr = self.readOdometry()
        if rotar and R != 0:
            R = self.encuentra_angulo(wXg, R)
        elif rotar:
            vect = np.array([wXg[0]-wXr[0], wXg[1]-wXr[1]])
            self.rota(math.atan2(vect[1], vect[0]), 0.5)
        wXr = self.readOdometry()
        # Posicion del robot respecto del mundo
        wTr = hom(wXr)
        # Hay que calcular la posicion del goal respecto del robot
        wTg = hom(wXg)

        # Posicion del mundo respecto del robot
        rTw = np.linalg.inv(wTr)

        # Matriz del goal respecto del robot
        rTg = rTw.dot(wTg)
        # Posicion del goal respecto del robot
        rXg = loc(rTg)
        x = rXg[0]
        y = rXg[1]

        # print("X Y R", x, y, R)
        dist = np.sqrt(x*x+y*y)
        v = dist/T
        if R != 0:
            w = v/R
        else:
            w = 0
        self.setSpeed(v, w)
        rXr = [0, 0, 0]

        # Bucle cerrado
        error_1 = 0
        Ki = 10
        Kp = 0.5
        accion_1 = np.array([v, w])
        i = 20
        while dist > error:
            print(dist)
            self.write_log("Objetivo a distancia: "+str(dist))
            velocity = self.readSpeed()
            # print("Velocity", velocity)
            error_v = np.array([v, w]) - np.array([velocity[0], velocity[1]])
            # error_v = -error_v

            accion = accion_1 + Kp * error_v + (Ki * .1 - Kp) * error_1
            # print(accion, error_v, v, w)
            error_1 = error_v
            accion_1 = accion
            self.setSpeed(accion[0], accion[1])
            rXr = self.simubot(accion, rXr, .1)
            rTr = hom(rXr)
            wTr_1 = wTr.dot(rTr)

            loc_robot = loc(wTr_1)
            dist_x = loc_robot[0] - wXg[0]
            dist_y = loc_robot[1] - wXg[1]
            dist = (np.sqrt(dist_x*dist_x + dist_y*dist_y))

            # Actualizamos la posicion del robot
            self.x = Value('d', loc_robot[0])
            self.y = Value('d', loc_robot[1])
            self.th = Value('d', loc_robot[2])

            self.POS.append(loc(wTr_1))
            self.V_acc.append(accion[0])
            self.W_acc.append(accion[1])
            self.V.append(velocity[0])
            self.W.append(velocity[1])

            i = i-1
        self.write_log("Objetivo alcanzado con error "+str(dist))
    """
        Simula el movimiento del robot desde xWR T segundos
        v, w = vc
    """

    def simubot(self, vc, xWR, T):
        if vc[1] == 0:   # w=0
            xRk = np.array([vc[0]*T, 0, 0])
        else:
            R = vc[0]/vc[1]
            # print(vc[1])

            dtitak = vc[1]*T
            titak = norm_pi(dtitak)
            # print(titak)
            xRk = np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])

        xWRp = loc(np.dot(hom(xWR), hom(xRk)))   # nueva localizaci�n xWR

        return xWRp

    """
        Lee la velocidad actual del robot
        Genera un error con probabilidad p
        Una vez que se ha producido un error:
            error del 10 % -> p = 0.75
            error del 20 % -> p = 0.15
            error del 30 % -> p = 0.1
    """

    def readSpeed(self, p=0.05):
        error = random.randint(0, 100)
        percentage = 0
        if(error >= (1-p)*100):
            self.error = True

            error = random.randint(0, 100)
            if(error >= 90):
                self.write_log("Error 30%")
                percentage = 0.3
            elif(error > 75):
                self.write_log("Error 20%")
                percentage = 0.2
            else:
                self.write_log("Error 10%")
                error = 0.1
        signo = random.randint(0, 1)

        signo = -1
        v = self.v.value + signo*(self.v.value * percentage)
        w = self.w.value + signo*(self.w.value * percentage)
        # self.setSpeed(v, w)
        #self.write_log("Velocidades actuales" + " "+str(v)+" "+str(w))
        return v, w

    """
        Devuelve la posicion actual
    """

    def readOdometry(self):
        """ Returns current value of odometry estimation """

        return self.x.value, self.y.value, self.th.value

    """
        Devuelve la lista de posiciones recorridas
    """

    def readPositions(self):
        return self.POS
    """
        Devuelve la lista de velocidades lineales leidas con readSpeed
    """

    def readV(self):
        return self.V
    """
        Devuelve la lista de velocidades angulares leidas con readSpeed
    """

    def readW(self):
        return self.W
    """
        Devuelve la lista de velocidades lineales declaradas con setSpeed
    """

    def readV_acc(self):
        return self.V_acc
    """
        Devuelve la lista de velocidades angulares declaradas con setSpeed
    """

    def readW_acc(self):
        return self.W_acc

    """
        ref -> path a la imagen de referencia
        img -> imagen que analizar

        Devuelve la media de la posicion de los keypoints de la imagen de referencia
        que coinciden en img
    """

    def match(self, ref, img):
        imgReference = cv2.imread(ref, cv2.IMREAD_COLOR)
        img = cv2.imread(img, cv2.IMREAD_COLOR)

        # Pasamos a blanco y negro para el feature extraction
        imgReference_gray = cv2.cvtColor(imgReference, cv2.COLOR_BGR2GRAY)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:  # CURRENT RASPBERRY opencv version is 2.4.9
            # Initiate ORB detector --> you could use any other detector, but this is the best performing one in this version
            binary_features = True

            detector = cv2.ORB()
        else:
            # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
            # but this is the best performing one in this version
            binary_features = True
            detector = cv2.BRISK_create()

        # find the keypoints and corresponding descriptors
        kp1, des1 = detector.detectAndCompute(imgReference_gray, None)
        kp2, des2 = detector.detectAndCompute(img_gray, None)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        good = matches

        MIN_MATCH_COUNT = 20          # initially

        MIN_MATCH_OBJECTFOUND = 10    # after robust check, to consider object-found
        matchesMask = None
        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32(
                [kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32(
                [kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            H_21, mask = cv2.findHomography(
                src_pts, dst_pts, cv2.RANSAC, 3.0)
            matchesMask = mask.ravel().tolist()
            num_robust_matches = np.sum(matchesMask)
            if num_robust_matches < MIN_MATCH_OBJECTFOUND:
                found = False
                print("NOT enough ROBUST matches found - %d (required %d)" %
                      (num_robust_matches, MIN_MATCH_OBJECTFOUND))
                exit(1)
        else:
            print("Not enough initial matches are found - %d (required %d)" %
                  (len(good), MIN_MATCH_COUNT))
            exit(1)

        acum = np.array([0, 0])
        for i, m in enumerate(matches):
            if matchesMask is None or (matchesMask is not None and mask[i]):
                coord = kp2[m.trainIdx].pt
                coord = np.array(coord)
                acum = acum + coord

        return acum / sum(matchesMask)

    """
        Devuelve una lista con todos los blobs de
        tal que color_min <= color_blob <= color_max

    """

    def return_blobs(self, image, color_min, color_max):
        img_BGR = cv2.imread(image)
        # Setup default values for SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # These are just examples, tune your own if needed
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 10000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Color
        params.filterByColor = False
        # not directly color, but intensity on the channel input
        # params.blobColor = 0
        params.filterByConvexity = False
        params.filterByInertia = False
        mask_color = cv2.inRange(img_BGR, color_min, color_max)

        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:
            detector = cv2.SimpleBlobDetector(params)
        else:
            detector = cv2.SimpleBlobDetector_create(params)

        # detector finds "dark" blobs by default, so invert image for results with same detector
        keypoints_color = detector.detect(255-mask_color)

        K = []
        for k in keypoints_color:
            K.append((k.pt[0], k.pt[1], k.size))
        return K

    """
        file_obstaculos es un fichero de texto con lineas que indican obstaculos:
            x, y, a -> la celda(x, y) tiene un obstaculo en su arista a
            valores de a: 7  0  1
                          6  xy 2
                          5  4  3
            mapa indica si el mapa es el A o el B
            goals son las casillas de final de planificacion

            Devuelve el objeto map2d para el plot
    """

    def recorrer_camino(self, file_obstaculos, mapa, goals):
        f = open(file_obstaculos)
        obstaculos = []
        for line in f:
            line = line.rstrip()
            line = line.split(",")
            obstaculos.append([int(line[0]), int(line[1]), int(line[2])])
        f.close()

        myMap = Map2D("mapa1.txt")

        # Partes a excluir segun sea la el mapa A o B
        parte_izq = [[0, 6], [2, 2]]
        parte_central = [[3, 6], [6, 3]]
        parte_dch = [[7, 6], [9, 2]]

        if mapa == 'A':
            parte_dch = [[7, 6], [9, 0]]
        elif mapa == 'B':
            parte_izq = [[0, 6], [2, 0]]
        else:
            print("El mapa tiene que ser A o B")
            exit(1)

        pos = self.readOdometry()
        origin = myMap._pos2cell(pos[0], pos[1])

        path = myMap.findPath(origin, goals, out_of_grid=[
            parte_izq, parte_central, parte_dch])

        # Generamos el mapa con los obstaculos aniadidos
        mapa_real = Map2D("mapa1.txt")
        for obstaculo in obstaculos:

            mapa_real.deleteConnection(
                obstaculo[0], obstaculo[1], obstaculo[2])

        posicion_recorrida = []

        # Se va comprobando si el camino se puede realizar sin obstaculos
        i = 0
        N = len(path)
        while i < N:
            posicion = path[i]
            if i == N-1:
                posicion_recorrida.append(posicion)
                self.write_log("Alcanzada ultima celda: " +
                               str(posicion[0])+" "+str(posicion[1]))
                if list(posicion) in list(goals):
                    self.write_log("¡Es un objetivo!")
            else:

                siguiente = path[i+1]
                conexion = arista(posicion, siguiente)
                if not mapa_real.isConnected(posicion[0], posicion[1], conexion):
                    # print("PATH anterior", path)
                    # Se ha detectado un obstaculo inesperado, se recalcula el camino
                    myMap.deleteConnection(posicion[0], posicion[1], conexion)

                    path = myMap.findPath(posicion, goals, out_of_grid=[
                        parte_izq, parte_central, parte_dch])
                    # print("PATH posterior", path)
                    i = 0
                    N = len(path)
                self.write_log("Celda actual: " +
                               str(posicion[0])+" "+str(posicion[1]))
                posicion_recorrida.append(posicion)

            i = i+1

        # Obtenemos las localizaciones de los robots
        localizaciones_robot = []
        for i, posicion in enumerate(posicion_recorrida):
            if i == len(posicion_recorrida)-1:

                x = 200 + 400 * posicion[0]
                y = 200 + 400 * posicion[1]
                theta = localizaciones_robot[i-1][2]

                localizaciones_robot.append([x, y, theta])
                self.POS.append([x, y, theta])

            else:
                siguiente = posicion_recorrida[i+1]
                conexion = arista(posicion, siguiente)
                x = 200 + 400 * posicion[0]
                y = 200 + 400 * posicion[1]
                thetas = [90, -1, 0, -1, -90, -1, 180]
                theta = thetas[conexion]
                localizaciones_robot.append([x, y, math.radians(theta)])
                self.POS.append([x, y, math.radians(theta)])
            self.x = Value('d', x)
            self.y = Value('d', y)
            self.th = Value('d', theta)

        return myMap
