#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
from MapLib import Map2D
import math
import matplotlib.pyplot as plt


def planificacion(robot, file_obstaculos, mapa, goals):

    localizaciones, myMap = robot.planificar(file_obstaculos, mapa, goals)
    return localizaciones, myMap
