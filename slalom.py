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
    try:
        if args.radioD < 0:
            print('d must be a positive value')
            exit(1)
        if args.radioD > 600:
            print("d is too high")
            exit(1)

        params = {}

        # Instantiate Odometry. Default value will be 0,0,0
        robot = Robot(params, init_position=[
                      600, (6*400 + 200), math.radians(270)])

        robot.rota(math.radians(180), .2)
        # robot.alcanza_objetivo(
        #     [600, (6*400 + 200), 3.14], 0, 200, 0.2)
        # Hay que cambiarlo para que entre de parametro de entrada
        robot.alcanza_objetivo(
            [600, (4*400 + 200), 3.14], 30, 0, 2)
        robot.alcanza_objetivo(
            [600, (2*400 + 200), 3.14], 30, 0, 2)
        POS = robot.readPositions()

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
        # robot.alcanza_objetivo([600, 2*400+200, 3.14], 150, A)

        # POS2 = robot.readPositions()

        # for elem in POS2:
        #     POS.append(elem)
        print(len(POS))
        myMap = Map2D("mapa1.txt")
        myMap.drawMapWithRobotLocations(
            POS, saveSnapshot=False)
        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        # robot.finish_log()

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        # robot.stopOdometry()
        print("Vaya")


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)",
                        type=float, default=400.0)
    args = parser.parse_args()

    main(args)
