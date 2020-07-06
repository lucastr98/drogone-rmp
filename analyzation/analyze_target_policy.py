import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

sigma = np.sqrt((1024 / 2) * (1024 / 2) + (768 / 2) * (768 / 2))
c = 0.005
alpha = 1
beta = 2

def s(x):
    x_norm = np.linalg.norm(x)
    h = x_norm + sigma * c * np.log(1 + np.exp(-2 * c * x_norm / sigma))
    return x / h

s_vals = []
x_vals = np.arange(-0, 1025, 0.01)
for x in x_vals:
    s_vals.append(s(x))


plt.plot(x_vals, s_vals, c='b')
c_handle = mpatches.Patch(color='b', label='c = 0.005')
plt.legend(handles=[c3_handle])
plt.grid()
plt.show()
