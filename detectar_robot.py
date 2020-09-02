#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
from Robot import Robot
import math

"""
    en Imagen aparecen los dos robots.
    El robot va hacia la salida que le corresponda según la posición
    del robot objetivo en dicha imagen
"""


def detectar_robot(robot, imagen, robot_objetivo, blob=False):
    pos_actual = robot.readOdometry()
    # Se avanza una casilla en linea recta para facilitar llegar a la casilla de salida
    siguiente = [pos_actual[0], pos_actual[1]+400, pos_actual[2]]
    robot.alcanza_objetivo(siguiente, 30, 0, 0.5, False)
    robot.alcanza_objetivo([5*400, 4*400+200, 0], 30, 0, 2)
    robot.rota(math.radians(90), 0.3)
    R2 = "R2-D2_s.png"
    BB = "BB8_s.png"

    if not blob:
        # Se localizan ambos robots
        coord_R2 = robot.match(R2, imagen)
        coord_BB = robot.match(BB, imagen)
    else:
        blobs_R2 = np.array(robot.return_blobs(
            imagen, (70, 5, 5), (255, 50, 50)))
        blobs_BB = np.array(robot.return_blobs(
            imagen, (10, 70, 100), (50, 128, 255)))
        print(len(blobs_R2))
        coord_R2 = np.mean(blobs_R2)
        coord_BB = np.mean(blobs_BB)

    R2_x = coord_R2[0]
    BB_x = coord_BB[0]

    # Casillas de salida izquierda y derecha
    fin_izq = [4, 6]
    fin_dch = [5, 6]
    fin = []

    # Se entiende que si la coordenada de un robot es menor a la del otro,
    # este primero se encuentra a la izquierda
    if robot_objetivo == "R2":
        if R2_x < BB_x:
            fin = fin_izq
            robot.write_log("Robot a la izquierda")
        else:
            fin = fin_dch
            robot.write_log("Robot a la derecha")

    elif robot_objetivo == "BB":
        if BB_x < R2_x:
            fin = fin_izq
            robot.write_log("Robot a la izquierda")
        else:
            fin = fin_dch
            robot.write_log("Robot a la derecha")

    else:
        print("El robot objetivo tiene que ser R2 o BB")
        exit(1)

    fin = [400*fin[0]+200, 400*fin[1]+200, 0]

    robot.alcanza_objetivo(fin, 80, 0, 2, True)
