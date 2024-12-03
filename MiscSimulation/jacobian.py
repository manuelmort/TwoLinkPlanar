import numpy as np
import matplotlib.pyplot as plt
import math
from TwoLink.TwoLinkInverse import inverse_kinematics_2link_elbow_up

a1 = 5
a2 = 4
x = 6
y = 6
theta1,theta2 = inverse_kinematics_2link_elbow_up(x,y,a1,a2)

xf = 7
yf = 2
t = 10

def nextPos(a1,a2,theta1,theta2,xf,yf,t):
    theta1_rad = np.deg2rad(theta1)
    theta2_rad = np.deg2rad(theta2)

    px = a1* np.cos(theta1_rad) + a2 * np.cos(theta1_rad + theta2_rad)
    py = a1* np.sin(theta1_rad) + a2 * np.sin(theta1_rad + theta2_rad)
    velocity_x = (xf - px)/t
    velocity_y = (yf - py)/t
    velocity_matrix = np.array([[velocity_x],[velocity_y]])


    #start while loop here

    print(f"\n\nStarting Initial End effector ({px},{py})")
    print(f"Destination End effector ({xf},{yf})\n\n")

    theta1Iterations = []
    theta2Iterations = []
    xIterations = []
    yIterations = []
    xcurr = px
    ycurr = py

    for i in range(10):
        # Create a matrix from a Python list
        matrix = np.array([[-a1 * np.sin(theta1_rad) - a2 * np.sin(theta1_rad + theta2_rad), -a2 * np.sin(theta1_rad + theta2_rad)],
                        [a1 * np.cos(theta1_rad) + a2 * np.cos(theta1_rad + theta2_rad), a2 * np.cos(theta1_rad + theta2_rad)]])
        inverse_matrix = np.linalg.inv(matrix)
        newAngle = np.dot(inverse_matrix,velocity_matrix)
        theta1Vel = newAngle[0]
        theta2Vel = newAngle[1]
       # print(theta1_rad,theta2_rad)
        theta1NewRad = theta1_rad + theta1Vel
        theta2NewRad = theta2_rad + theta2Vel
        #print(theta1NewRad,theta2NewRad)


        xcurr = a1 * np.cos(theta1NewRad)+ a2 * np.cos(theta1NewRad + theta2NewRad)
        ycurr = a1 * np.sin(theta1NewRad) + a2 * np.sin(theta1NewRad + theta2NewRad)

        theta1_rad = theta1NewRad[0]
        theta2_rad = theta2NewRad[0]
        
        #storing every single iteration in appropriate array, this will be important to animate every single step
        xIterations.append(xcurr[0]),yIterations.append(ycurr[0]),theta1Iterations.append(theta1_rad),theta2Iterations.append(theta2_rad)
        #print(xcurr,ycurr)

        #plot_robot_configuration(theta1NewDeg, theta2NewDeg, a1, a2)
        



    return theta1_rad, theta2_rad,theta1Iterations,theta2Iterations,xIterations,yIterations


nextPos(a1,a2,theta1,theta2,xf,yf,t)

