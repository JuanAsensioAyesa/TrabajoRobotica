#!/usr/bin/python
# -*- coding: UTF-8 -*-
import argparse
import numpy as np
import time
from Robot import Robot


def detectar_blob(robot, imagen):

    blobs = robot.return_blobs(imagen, (10, 10, 100), (50, 50, 255))
    return sorted(blobs, key=lambda x: x[2])
