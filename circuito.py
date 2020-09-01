from Robot import Robot
from detectar_blob import detectar_blob
from detectar_robot import detectar_robot
import argparse
import numpy as np
import math
from slalom import slalom
from MapLib import Map2D
from planificacion import planificacion

if __name__ == "__main__":

    # get and parse arguments passed to main
    # Add as many args as you need ...
    ap = argparse.ArgumentParser()
    ap.add_argument("-r", "--robots", default="test2.jpg",
                    help="path de la imagen de los robots")
    ap.add_argument("-b", "--ball", default="red_blue.jpg",
                    help="path de la imagen de la bola roja")
    ap.add_argument("-R", "--radio", default=400,
                    help="radio para realizar el slalom")
    ap.add_argument("-s", "--start", default="A",
                    help="Inicio del circuito (A o B)")

    args = ap.parse_args()

    posicion_inicial_A = np.array([400+200, 6*400+200, math.radians(270)])
    posicion_intermedia_A = np.array([400+200, 4*400+200, math.radians(270)])
    posicion_final_A = np.array([400+200, 2*400+200, math.radians(270)])

    posicion_inicial_B = np.array([8*400+200, 6*400+200, math.radians(270)])
    posicion_intermedia_B = np.array([8*400+200, 4*400+200, math.radians(270)])
    posicion_final_B = np.array([8*400+200, 2*400+200, math.radians(270)])

    posicion = []
    intermedio = []
    final = []

    if args.start == "A":
        posicion = posicion_inicial_A
        intermedio = posicion_intermedia_A
        final = posicion_final_A

    elif args.start == "B":
        posicion = posicion_inicial_B
        intermedio = posicion_intermedia_B
        final = posicion_final_B

    else:
        print("El mapa tiene que ser A o B")
        exit(1)
    goals = [[3, 2], [6, 2]]
    robot = Robot(init_position=posicion)
    slalom(robot, posicion, intermedio, final, args.radio)
    posiciones_planificacion, myMap = planificacion(
        robot, "obstaculos.txt", args.start, goals)

    blobs = detectar_blob(robot, args.ball)
    posiciones_robot = detectar_robot(robot, args.robots, "BB")

    posiciones_robot = robot.readPositions()

    print(blobs[0])

    for pos in posiciones_planificacion:
        posiciones_robot.append(pos)

    myMap.drawMapWithRobotLocations(posiciones_robot, saveSnapshot=False)