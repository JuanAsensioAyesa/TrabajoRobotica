#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
from MapLib import Map2D
import math
import matplotlib.pyplot as plt

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


def main(args):
    file = args.file
    f = open(file)
    obstaculos = []
    for line in f:
        line = line.rstrip()
        line = line.split(",")
        obstaculos.append([int(line[0]), int(line[1]), int(line[2])])
    f.close()
    mapa = args.mapa
    myMap = Map2D("mapa1.txt")

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

    origin = [1, 2]
    goals = [[3, 2], [6, 2]]
    path = myMap.findPath(origin, goals, out_of_grid=[
        parte_izq, parte_central, parte_dch])

    # Generamos el mapa con los obstaculos aniadidos
    mapa_real = Map2D("mapa1.txt")
    for obstaculo in obstaculos:
        mapa_real.deleteConnection(obstaculo[0], obstaculo[1], obstaculo[2])

    posicion_recorrida = []

    i = 0
    N = len(path)
    while i < N:
        posicion = path[i]
        if i == N-1:
            posicion_recorrida.append(posicion)
        else:

            siguiente = path[i+1]
            conexion = arista(posicion, siguiente)
            if not mapa_real.isConnected(posicion[0], posicion[1], conexion):
                print("PATH anterior", path)
                myMap.deleteConnection(posicion[0], posicion[1], conexion)

                path = myMap.findPath(posicion, goals, out_of_grid=[
                    parte_izq, parte_central, parte_dch])
                print("PATH posterior", path)
                i = 0
                N = len(path)

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

        else:
            siguiente = posicion_recorrida[i+1]
            conexion = arista(posicion, siguiente)
            x = 200 + 400 * posicion[0]
            y = 200 + 400 * posicion[1]
            thetas = [90, -1, 0, -1, -90, -1, 180]
            theta = thetas[conexion]
            localizaciones_robot.append([x, y, math.radians(theta)])

    myMap.drawMapWithRobotLocations(localizaciones_robot, saveSnapshot=False)


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapa", help="Tipo de mapa , A o B",
                        type=str, default='A')
    parser.add_argument("-f", "--file", help="Nombre del fichero con los obstaculos",
                        type=str, default='obstaculos.txt')
    args = parser.parse_args()

    main(args)
