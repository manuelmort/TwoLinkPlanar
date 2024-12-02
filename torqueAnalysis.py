import numpy as np
import matplotlib.pyplot as plt

# Data for Standard PID
torques_pid = np.array([
    [33.64129485, -6.94220459],
    [33.04427076, -4.63689242],
    [34.14504141, -3.10384725],
    [35.42356076, -1.76638769],
    [36.50930065, -0.62134565],
    [37.33716052,  0.32434624],
    [37.92461863,  1.08379726],
    [38.31028753,  1.68083174],
    [38.53482583,  2.14195728],
    [38.63476074,  2.49225457],
    [38.64083827,  2.75371982],
    [38.57806906,  2.94486882],
    [38.46637832,  3.0809194 ],
    [38.32141042,  3.1741966 ],
    [38.155304,    3.23458774],
    [37.97737121,  3.26997091],
    [37.79466635,  3.2865887 ],
    [37.61245094,  3.28936204]
])

# Data for Log-Scale PID
torques_log = np.array([
    [30.68104595, -11.27758274],
    [31.40509623, -8.38299769],
    [32.60308795, -5.73628825],
    [34.25400383, -3.19857101],
    [36.08198655, -1.01130104],
    [37.79484133,  0.64717501],
    [39.1914205,   1.80876081],
    [40.20836206,  2.63117737],
    [40.89167428,  3.23887022],
    [41.31852086,  3.68859594],
    [41.55102154,  4.01047894],
    [41.63258473,  4.22886735],
    [41.59470241,  4.36447562],
    [41.46147044,  4.43422548],
    [41.25200379,  4.45175085],
    [40.98191001,  4.42807957],
    [40.66420912,  4.37199091],
    [40.31010014,  4.2914245 ],
    [39.92974404,  4.1927186 ],
    [39.53205624,  4.08104876],
    [39.12494709,  3.96080507],
    [38.71549162,  3.83572907],
    [38.31000785,  3.70900571],
    [37.91408314,  3.58332747]
])

# Plot for Standard PID using points
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.scatter(range(len(torques_pid)), torques_pid[:, 0], label='Tau1 (Standard PID)', color='blue')
plt.scatter(range(len(torques_pid)), torques_pid[:, 1], label='Tau2 (Standard PID)', color='orange')
plt.xlabel('Iteration')
plt.ylabel('Torque (Nm)')
plt.title('Standard PID Torque (Points)')
plt.legend()
plt.grid(True)

# Plot for Log-Scale PID using points
plt.subplot(1, 2, 2)
plt.scatter(range(len(torques_log)), torques_log[:, 0], label='Tau1 (Log-Scale PID)', color='green')
plt.scatter(range(len(torques_log)), torques_log[:, 1], label='Tau2 (Log-Scale PID)', color='purple')
plt.xlabel('Iteration')
plt.ylabel('Torque (Nm)')
plt.title('Log-Scale PID Torque (Points)')
plt.legend()
plt.grid(True)

# Show plots
plt.tight_layout()
plt.show()
