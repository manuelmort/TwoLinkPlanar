import numpy as np
from flask import Flask, render_template,jsonify
import matplotlib.pyplot as plt
import math
from jacobian import nextPos
from twolinkplanar import inverse_kinematics_2link_elbow_down, inverse_kinematics_2link_elbow_up
from plotConfig import plot_robot_configuration

a1 = 5
a2 = 4
x = 6
y = 6

xf = 7
yf = 2
t = 10

theta1Iterations = []
theta2Iterations = []
xIterations = []
yIterations = []



#plot_robot_configuration(theta1, theta2, a1, a2)


theta1_deg_down, theta2_deg_down = inverse_kinematics_2link_elbow_down(x, y, a1, a2)
theta1_deg_up, theta2_deg_up = inverse_kinematics_2link_elbow_up(x, y, a1, a2)

newTheta1Up,newTheta2Up,theta1Iterations,theta2Iterations,xIterations,yIterations = nextPos(a1,a2,theta1_deg_up,theta2_deg_up,xf,yf,t)
newTheta1Down,newTheta2Down,theta1Iterations,theta2Iterations,xIterations,yIterations = nextPos(a1,a2,theta1_deg_down,theta2_deg_down,xf,yf,t)
print(xIterations)
print(yIterations)

plt.figure()

plot_robot_configuration(theta1_deg_up, theta2_deg_up, a1, a2)
plot_robot_configuration(theta1_deg_down, theta2_deg_down, a1, a2)

plot_robot_configuration(np.rad2deg(newTheta1Up), np.rad2deg(newTheta2Up), a1, a2)
plot_robot_configuration(np.rad2deg(newTheta1Down), np.rad2deg(newTheta2Down), a1, a2)


plt.title("2-Link Manipulator - Elbow Down and Elbow Up Configurations")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend(["Elbow Down", "Elbow Up"])
plt.grid()
plt.axis("equal")
#plt.show()


app = Flask(__name__)

@app.route('/home', methods=['GET'])
def home():
    armData = {
        'theta1':theta1_deg_up,'theta2':theta2_deg_up,
        'a1':a1,
        'a2':a2,
        'x':x,
        'y':y,
        'xf':xf,
        'yf':yf,
        'newTheta1Up':newTheta1Up,
        'newTheta2Up':newTheta2Up,
        'theta1Iterations': theta1Iterations,
        'theta2Iterations': theta2Iterations,
        'xIterations': xIterations,
        'yIterations': yIterations

    }
    return jsonify(armData)

if __name__ == '__main__':
    app.run(debug=True,host='127.0.0.1', port=5000)
