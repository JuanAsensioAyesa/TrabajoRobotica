#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot


"""
    Devuelve la lista ordenada de los blobs de color rojo detectados por el 
    robot
    La lista estara ordenada de menor a mayor tamanio del blob
"""


def detectar_blob(robot, imagen):

    blobs = robot.return_blobs(imagen, (10, 10, 100), (50, 50, 255))
    ordenados = sorted(blobs, key=lambda x: x[2])
    mas_grande = ordenados[-1]
    robot.write_log("Blob mas grande x: " +
                    str(mas_grande[0])+" y: "+str(mas_grande[1])+" size: "+str(mas_grande[2]))
    return ordenados
