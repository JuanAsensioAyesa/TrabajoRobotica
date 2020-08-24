#!/usr/bin/python
# -*- coding: UTF-8 -*-
# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''

# import brickpi3 # import the BrickPi3 drivers

import time     # import the time library for the sleep function
import sys
import random  # Simular perturbaciones
import numpy as np
import math
# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

# Posicion a polares segun se indica en las transparencias


def posicion_a_polar(x):
    dx = x[0]
    dy = x[1]
    theta = x[2]
    p = np.sqrt(dx * dx + dy * dy)
    beta = norm_pi(math.atan2(dy, dx) + np.pi)
    alpha = beta - theta
    return np.array([p, alpha, beta])


def loc(T, degrees=False):
    T = np.array(T)
    nx = T[0, 0]
    ny = T[1, 0]
    if degrees:
        nx = math.radians(nx)
        ny = math.radians(ny)
    theta = math.atan2(ny, nx)
    px = T[0, 2]
    py = T[1, 2]
    return np.array([px, py, theta])


# Funcion hom que crea la matriz T a partir de una posicion
def hom(x, degrees=False):
    x = np.array(x)
    dx = x[0]
    dy = x[1]
    theta = x[2]

    if degrees:
        theta = math.radians(theta)
    sin = math.sin(theta)
    cos = math.cos(theta)
    m = [[cos, -sin, dx], [sin, cos, dy], [0, 0, 1]]
    return np.array(m)


def norm_pi(th):
    th_norm = th
    while th_norm > np.pi:
        th_norm -= 2 * np.pi

    while th_norm < -np.pi:
        th_norm += 2 * np.pi

    return th_norm


class Robot:
    def __init__(self,  params, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        # self.R = ??
        # self.L = ??
        # self. ...

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        # self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        # self.BP.offset_motor_encoder(self.BP.PORT_B,
        #    self.BP.get_motor_encoder(self.BP.PORT_B))
        # self.BP.offset_motor_encoder(self.BP.PORT_C,
        #    self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])
        # boolean to show if odometry updates are finished
        self.finished = Value('b', 1)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        # self.lock_odometry = Lock()
        # self.lock_odometry.acquire()
        # print('hello world', i)
        # self.lock_odometry.release()

        # odometry update period --> UPDATE value!
        self.P = 1.0

        # Velocidades lineal y angular
        self.v = 0
        self.w = 0

        # Listas de log
        self.V = []
        self.W = []
        self.V_acc = []
        self.W_acc = []
        self.POS = [init_position]
        self.error = False

    def setSpeed(self, v, w):
        """ To be filled - These is all dummy sample code """
        # print("setting speed to %.2f %.2f" % (v, w))

        self.v = v
        self.w = w

    # Rota en su posicion hasta alcanzar el angulo theta en T segundos
    def rota(self, theta, T):
        pos = self.readOdometry()
        theta = norm_pi(theta)

        theta_R = norm_pi(pos[2])
        dif = theta - theta_R
        w = dif/T

        while abs(pos[2] - theta) > 0.02:
            pos = self.simubot([0, w], pos, .1)
            # print(math.degrees(pos[2]), math.degrees(theta))
            self.POS.append(pos)
        self.th = Value('d', pos[2])
    # Alcanza un objetivo con un error <= error
    # Realiza una circunferencia de radio R
    # Alcanza el objetivo en T segundos

    def alcanza_objetivo(self, wXg, error, R, T):
        wXr = self.readOdometry()
        # Posicion del robot respecto del mundo
        wTr = hom(wXr)
        # Hay que calcular la posicion del goal respecto del robot
        wTg = hom(wXg)

        # Posicion del mundo respecto del robot
        rTw = np.linalg.inv(wTr)

        # Matriz del goal respecto del robot
        rTg = rTw.dot(wTg)
        # Posicion del goal respecto del robot
        rXg = loc(rTg)
        x = rXg[0]
        y = rXg[1]

        R = (np.power(x, 2)+np.power(y, 2))/(2*y)
        print("X Y R", x, y, R)
        dist = np.sqrt(x*x+y*y)
        v = dist/T
        w = v/R
        self.setSpeed(v, w)
        rXr = [0, 0, 0]
        i = 600

        # Bucle cerrado
        error_1 = 0
        Ki = 10
        Kp = 0.5
        accion_1 = np.array([v, w])
        while dist > error:
            velocity = self.readSpeed()
            print("Velocity", velocity)
            error_v = np.array([velocity[0], velocity[1]]) - np.array([v, w])
            error_v = -error_v
            #print(v, w)

            accion = accion_1 + Kp * error_v + (Ki * .1 - Kp) * error_1
            print(accion, error_v, v, w)
            error_1 = error_v
            accion_1 = accion
            self.setSpeed(accion[0], accion[1])
            rXr = self.simubot(accion, rXr, .1)
            rTr = hom(rXr)
            wTr_1 = wTr.dot(rTr)
            self.POS.append(loc(wTr_1))
            i = i-1
            loc_robot = loc(wTr_1)
            dist_x = loc_robot[0] - wXg[0]
            dist_y = loc_robot[1] - wXg[1]
            dist = (np.sqrt(dist_x*dist_x + dist_y*dist_y))
            self.V.append(velocity[0])
            self.W.append(velocity[1])
            self.V_acc.append(accion[0])
            self.W_acc.append(accion[1])
        self.x = Value('d', loc_robot[0])
        self.y = Value('d', loc_robot[1])
        self.th = Value('d', loc_robot[2])

    def simubot(self, vc, xWR, T):
        if vc[1] == 0:   # w=0
            xRk = np.array([vc[0]*T, 0, 0])
        else:
            R = vc[0]/vc[1]
            # print(vc[1])

            dtitak = vc[1]*T
            titak = norm_pi(dtitak)
            # print(titak)
            xRk = np.array([R*np.sin(titak), R*(1-np.cos(titak)), titak])

        xWRp = loc(np.dot(hom(xWR), hom(xRk)))   # nueva localizaci�n xWR
        return xWRp

    # Genera un error aleatorio
    # Probabilidad de error = 0.1 (Cada vez que se lee)
    # Una vez hay error:
    # Error de un 10% -> probabilidad 70%
    # Error de un 20% -> probabilidad 25%
    # Error de un 30% -> probabilidad 5%
    def readSpeed(self):
        error = random.randint(0, 100)
        percentage = 0
        if(error >= 90):
            self.error = True
            print("ERROR")
            error = random.randint(0, 100)
            if(error >= 90):
                percentage = 0.5
            elif(error >= 75):
                percentage = 0.2
            else:
                error = 0.1
        signo = random.randint(0, 1)

        signo = -1
        v = self.v + signo*(self.v * percentage)
        w = self.w + signo*(self.w*percentage)
        #self.setSpeed(v, w)
        return v, w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def readPositions(self):
        return self.POS

    def readV(self):
        return self.V

    def readW(self):
        return self.W

    def readV_acc(self):
        return self.V_acc

    def readW_acc(self):
        return self.W_acc
        # def startOdometry(self):
        #     """ This starts a new process/thread that will be updating the odometry periodically """
        #     self.finished.value = False
        #     # additional_params?))
        #     self.p = Process(target=self.updateOdometry, args=())
        #     self.p.start()
        #     print("PID: ", self.p.pid)

        # # You may want to pass additional shared variables besides the odometry values and stop flag
        # def updateOdometry(self):  # , additional_params?):
        #     """ To be filled ...  """

        #     while not self.finished.value:
        #         # current processor time in a floating point value, in seconds
        #         tIni = time.clock()

        #         # compute updates

        #         ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
        #         sys.stdout.write("Update of odometry ...., X=  %d, \
        #             Y=  %d, th=  %d \n" % (self.x.value, self.y.value, self.th.value))
        #         #print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )

        #         # update odometry uses values that require mutex
        #         # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

        #         # Operations like += which involve a read and write are not atomic.
        #         # with self.x.get_lock():
        #         #     self.x.value += 1

        #         # # to "lock" a whole set of operations, we can use a "mutex"
        #         # self.lock_odometry.acquire()
        #         # # self.x.value+=1
        #         # self.y.value += 1
        #         # self.th.value += 1
        #         # self.lock_odometry.release()

        #         # save LOG
        #         # Need to decide when to store a log with the updated odometry ...

        #         ######## UPDATE UNTIL HERE with your code ########

        #         tEnd = time.clock()
        #         time.sleep(self.P - (tEnd-tIni))

        #     #print("Stopping odometry ... X= %d" %(self.x.value))
        #     sys.stdout.write("Stopping odometry ... X=  %.2f, \
        #             Y=  %.2f, th=  %.2f \n" % (self.x.value, self.y.value, self.th.value))

        # # Stop the odometry thread.

        # def stopOdometry(self):
        #     self.finished.value = True
        #     # self.BP.reset_all()
