import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.pyplot import cm

d_drone = 0.4
f_x = 1140

z_distances = np.linspace(0, 20, 1000)
tol_d = np.linspace(1, 5, 5)

noise_d_complete = []
for tol in tol_d:
    noise_d = []
    for z_distance in z_distances:
        noise_d.append(abs(d_drone * f_x * abs(1 / (tol + f_x * (d_drone / z_distance)) - 1 / (f_x * d_drone / z_distance))))
    noise_d_complete.append(noise_d)

n = len(noise_d_complete)
color = iter(cm.rainbow(np.linspace(0, 1, n)))

for i in range(len(noise_d_complete)):
    c = next(color)
    plt.plot(z_distances, noise_d_complete[i], c=c, label='tol = %i' %tol_d[i])

plt.xlabel('distance in z (camera frame) [m]')
plt.ylabel('stddev to use in noise calculation [m]')
plt.grid()
plt.legend()
plt.show()
