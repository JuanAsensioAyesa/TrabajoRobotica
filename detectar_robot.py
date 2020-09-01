#!/usr/bin/python
# -*- coding: UTF-8 -*-
import numpy as np
from Robot import Robot


def detectar_robot(robot, imagen, robot_objetivo):
    R2 = "R2-D2_s.png"
    BB = "BB8_s.png"

    coord_R2 = robot.match(R2, imagen)
    coord_BB = robot.match(BB, imagen)

    R2_x = coord_R2[0]
    BB_x = coord_BB[0]

    fin_izq = [4, 6]
    fin_dch = [5, 6]
    fin = []
    if robot_objetivo == "R2":
        if R2_x < BB_x:
            fin = fin_izq
        else:
            fin = fin_dch

    elif robot_objetivo == "BB":
        if BB_x < R2_x:
            fin = fin_izq
        else:
            fin = fin_dch

    else:
        print("El robot objetivo tiene que ser R2 o BB")
        exit(1)

    fin = [400*fin[0]+200, 400*fin[1]+200, 0]
    pos_actual = robot.readOdometry()
    siguiente = [pos_actual[0], pos_actual[1]+400, pos_actual[2]]
    robot.alcanza_objetivo(siguiente, 30, 0, 0.5, False)
    robot.alcanza_objetivo(fin, 80, 0, 2, True)
