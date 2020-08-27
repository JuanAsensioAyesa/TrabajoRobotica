#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import cv2
import numpy as np
import time
from Robot import Robot


def main(args):
    try:

        robot = Robot()
        blobs = robot.return_blobs(args.image, (10, 10, 100), (50, 50, 255))
        print(blobs)
        print(len(blobs))
    except KeyboardInterrupt:
        # except the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        print("Programa interrumpido")


if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", default="many.jpg",
                    help="path to the input image")

    args = ap.parse_args()

    main(args)
