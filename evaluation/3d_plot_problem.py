import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def circle(r, phi, theta):
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    point = np.matrix([[x],
                       [y],
                       [z]])
    return point


r = 1
A_uv = np.matrix([[1, 0, -0.6],
                  [0, 1, -0.6],
                  [-0.6, -0.6, 0.72]])
A_d = np.matrix([[0.07627119, 0.07627119, 0.25423729],
                 [0.07627119, 0.07627119, 0.25423729],
                 [0.25423729, 0.25423729, 0.84745763]])
x = []
y = []
z = []
x_stretched_uv = []
y_stretched_uv = []
z_stretched_uv = []
x_stretched_d = []
y_stretched_d = []
z_stretched_d = []

for i in np.linspace(0, 2 * np.pi, 200):
    for j in np.linspace(-np.pi, np.pi, 200):
        point = circle(r, i, j)
        x.append(point[0, 0])
        y.append(point[1, 0])
        z.append(point[2, 0])
        point_stretched_uv = A_uv.dot(point)
        x_stretched_uv.append(point_stretched_uv[0, 0])
        y_stretched_uv.append(point_stretched_uv[1, 0])
        z_stretched_uv.append(point_stretched_uv[2, 0])
        point_stretched_d = A_d.dot(point)
        x_stretched_d.append(point_stretched_d[0, 0])
        y_stretched_d.append(point_stretched_d[1, 0])
        z_stretched_d.append(point_stretched_d[2, 0])


# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# ax.plot(x, y, z, c='y')
# ax.plot(x_stretched_uv, y_stretched_uv, z_stretched_uv, c='r')
# ax.plot(x_stretched_d, y_stretched_d, z_stretched_d, c='b')
# # ax.plot(x, y, z, c='y')
# ax.axis('equal')
#
# plt.show()


from mayavi import mlab
fig = mlab.figure()

ax_ranges = [-1.5, 1.5, -1.5, 1.5, -1.5, 1.5]
ax_scale = [1.0, 1.0, 1.0]
ax_extent = ax_ranges * np.repeat(ax_scale, 2)


surf1 = mlab.surf(x, y, z, colormap='Blues')
surf1.actor.actor.scale = ax_scale
mlab.view(60, 74, 17, [-2.5, -4.6, -0.3])
mlab.outline(surf1, color=(.7, .7, .7), extent=ax_extent)
mlab.axes(surf1, color=(.7, .7, .7), extent=ax_extent, ranges=ax_ranges, xlabel='x', ylabel='y', zlabel='z')
