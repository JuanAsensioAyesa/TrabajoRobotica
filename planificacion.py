#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot
from MapLib import Map2D
import math
import matplotlib.pyplot as plt


def main(args):
    mapa = args.mapa
    myMap = Map2D("mapa1.txt")

    myMap.deleteConnection(4, 2, 6)
    myMap.deleteConnection(3, 1, 0)
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

    myMap.fillCostMatrix([[3, 2], [6, 2]], out_of_grid=[
                         parte_izq, parte_central, parte_dch])
    myMap.drawMap()
    print(myMap.isConnected(0, 0, 6))
    conn_matrix = myMap.connectionMatrix
    cost_matrix = myMap.costMatrix


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mapa", help="Tipo de mapa , A o B",
                        type=str, default='A')
    args = parser.parse_args()

    main(args)
