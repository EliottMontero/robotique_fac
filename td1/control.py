import math
import numpy as np


def updateRobotPos(robotPos, mousePos):

    alpha = (math.pi/500)
    x = robotPos[0]
    y = robotPos[1]



#### VERSION BRUTE FORCE #########

    # r = math.sqrt(x*x + y*y)
    # teta = math.atan2(y,x)
    #
    # teta = teta + alpha
    #
    # x = math.cos(teta) * r
    # y = math.sin(teta) * r
    #
    # robotPos[0] = x
    # robotPos[1] = y


#### VERSION UN PEU PLUS SMART #########

    #robotPos[0] = math.cos(alpha)*x - math.sin(alpha)*y
    #robotPos[1] = math.sin(alpha)*x + math.cos(alpha)*y

#### VERSION MATRICIELLE ORIGINE ##########

    # vect = np.matrix(robotPos).T
    #
    # matrix = np.array([[math.cos(alpha),-math.sin(alpha)],[math.sin(alpha),math.cos(alpha)]])
    #
    # end_matrix = matrix.dot(vect)
    #
    # robotPos[0] = end_matrix[0]
    # robotPos[1] = end_matrix[1]


#### VERSION MATRICIELLE SOURIS (HOMOGENE) ##########

    mx = mousePos[0]
    my = mousePos[1]

    vect = np.matrix([x,y,1]).T
    R_alpha = np.array([[math.cos(alpha),-math.sin(alpha),0],[math.sin(alpha),math.cos(alpha),0],[0,0,1]])
    T_mouse = np.array([[1, 0, mx], [0, 1, my], [0, 0, 1]])
    T_mouse_inv = np.array([[1, 0, -mx], [0, 1, -my], [0, 0, 1]])

    end_matrix = T_mouse.dot(R_alpha.dot(T_mouse_inv.dot(vect)))


    robotPos[0] = end_matrix[0]
    robotPos[1] = end_matrix[1]

    ### REDUCTION DISTANCE AU POINT ####


    return robotPos
