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
    ap.add_argument("-g", "--goal", default="R2",
                    help="Robot objetivo (R2 o BB)")

    args = ap.parse_args()
    args.radio = int(args.radio)

    # Posiciones iniciales,intermedia y finales segun el mapa A o B
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

    # Posiciones finales para la planificacion del camino
    goals = [[3, 2], [6, 2]]
    # Inicializa al Robot en su posicion inicial
    robot = Robot(init_position=posicion)
    robot.init_odometria()

    # Realiza el slalom correspondiente
    slalom(robot, posicion, intermedio, final, args.radio)

    myMap = planificacion(
        robot, "obstaculos.txt", args.start, goals)

    # Detecta los blobs
    blobs = detectar_blob(robot, args.ball)

    # Detecta el robot y se dirige a su salida
    detectar_robot(robot, args.robots, args.goal)

    # Lee todas las posiciones que ha recorrido el robot
    posiciones_robot = robot.readPositions()

    # Printea las coordenadas y el tamanio del mayor blob rojo
    print(blobs[-1])

    myMap.drawMapWithRobotLocations(posiciones_robot, saveSnapshot=False)
    robot.stop_odometria()
