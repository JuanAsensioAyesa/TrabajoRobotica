#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
from MapLib import Map2D
import math
import matplotlib.pyplot as plt


"""
    Realiza un slalom,con radio R, alcanzando en primer lugar la posicion intermedia,y 
    seguidamente la posicion final
"""


def slalom(robot, inicial, intermedia, final, R):

    robot.alcanza_objetivo(
        intermedia, 70, R, 2, )

    # Ponemos rotar a false para que realice una trayectoria mas "natural"
    robot.alcanza_objetivo(
        final, 70, -R, 2, rotar=False)

    plt.figure("V")
    V = robot.readV()
    print(len(V))
    V_acc = robot.readV_acc()
    plt.plot(V, label='V')
    plt.plot(V_acc, color='red', label='V_acc')
    plt.legend()
    plt.show()

    plt.figure("W")
    W = robot.readW()
    plt.plot(W)
    plt.show()
