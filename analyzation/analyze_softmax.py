import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

sigma = np.sqrt((1024 / 2) * (1024 / 2) + (768 / 2) * (768 / 2))
# sigma = np.sqrt((1024) * (1024) + (768) * (768))

def s(x, c):
    x_norm = np.linalg.norm(x)
    # h = x_norm + c * np.log(1 + np.exp(-2 * c * x_norm))
    h = x_norm + sigma * c * np.log(1 + np.exp(-2 * c * x_norm / sigma))
    return x / h

the higher c, the flatter the change from -1 to 1
c -> 1 gives us a limit on the one side and c -> 0 on the other side
c1 = 0.00005
c2 = 0.0005
c3 = 0.005
c4 = 0.05
c5 = 0.5
s_vals1 = []
s_vals2 = []
s_vals3 = []
s_vals4 = []
s_vals5 = []
x_vals = np.arange(-1024, 1025, 0.01)
x_vals = np.arange(-0, 1025, 0.01)
# x_vals = np.arange(-2.0, 2.0, 0.01)
for x in x_vals:
    x = x * 0.001
    x = x * 0.000264583
    s_vals1.append(s(x, c1))
    s_vals2.append(s(x, c2))
    s_vals3.append(s(x, c3))
    s_vals4.append(s(x, c4))
    s_vals5.append(s(x, c5))

plt.plot(x_vals, s_vals1, c='r')
plt.plot(x_vals, s_vals2, c='g')
plt.plot(x_vals, s_vals3, c='b')
plt.plot(x_vals, s_vals4, c='y')
plt.plot(x_vals, s_vals5, c='orange')
c1_handle = mpatches.Patch(color='r', label='c = 0.00005')
c2_handle = mpatches.Patch(color='g', label='c = 0.0005')
c3_handle = mpatches.Patch(color='b', label='c = 0.005')
c4_handle = mpatches.Patch(color='y', label='c = 0.05')
c5_handle = mpatches.Patch(color='orange', label='c = 0.5')
plt.legend(handles=[c1_handle, c2_handle, c3_handle, c4_handle, c5_handle])
plt.grid()
plt.show()
