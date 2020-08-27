#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
import numpy as np
import time
from Robot import Robot


def main(args):
    try:
        R2 = "R2-D2_s.png"
        BB = "BB8_s.png"
        robot = Robot()
        coord_R2 = robot.match(R2, args.image)
        coord_BB = robot.match(BB, args.image)

        print("R2", coord_R2)
        print("BB", coord_BB)

    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        print("Programa interrumpido")


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", default="test2.jpg",
                    help="path to the input image")

    args = ap.parse_args()

    main(args)
