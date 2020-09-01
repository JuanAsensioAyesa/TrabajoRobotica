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
    Realiza la planificacion del robot par el camino segun los obstaculos que aparezcan
    Devuelve las localizaciones del robot y el objeto mapa2D para el plot
"""


def planificacion(robot, file_obstaculos, mapa, goals):

    localizaciones, myMap = robot.planificar(file_obstaculos, mapa, goals)
    return localizaciones, myMap
