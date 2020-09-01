#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
from MapLib import Map2D
import math
import matplotlib.pyplot as plt


def slalom(robot, inicial, intermedia, final, R):

    robot.alcanza_objetivo(
        intermedia, 30, R, 2, )
    robot.alcanza_objetivo(
        final, 30, -R, 2, rotar=False)

    plt.figure("V")
    V = robot.readV()
    V_acc = robot.readV_acc()
    plt.plot(V)
    plt.plot(V_acc, color='red')
    plt.show()

    plt.figure("W")
    W = robot.readW()
    plt.plot(W)
    plt.show()
