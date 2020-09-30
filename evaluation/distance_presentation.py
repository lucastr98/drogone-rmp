import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rosbag

################################## DISTANCE ###################################
pts = []
for i in range(100):
    pts.append((i + 1) * 0.01)
pts.reverse()

fig = plt.figure()
ax = fig.add_subplot(1, 5, 1)

def animate(i):
    if(i < len(pts)):
        ax.clear()
        ax.bar(" ", pts[i], color='brown')
        ax.set_ylim(0, 1)
        if(len(str(i)) == 1):
            fig.savefig('plot-000' + str(i) + '.png')
        elif(len(str(i)) == 2):
            fig.savefig('plot-00' + str(i) + '.png')
        elif(len(str(i)) == 3):
            fig.savefig('plot-0' + str(i) + '.png')
        elif(len(str(i)) == 4):
            fig.savefig('plot-' + str(i) + '.png')
    else:
        print("finished")

ani = animation.FuncAnimation(fig, animate, interval=10)

# ############################### DISTANCE2GROUND ###############################
# pts = []
# appender = 1.015
# for i in range(100):
#     if(i < 50 + 5):
#         appender -= 0.015
#         pts.append(appender)
#     elif(i >= 50 + 5 and i < 57 + 5):
#         appender += 0.01
#         pts.append(appender)
#     elif(i >= 57 + 5 and i < 70 + 5):
#         appender += 0.005
#         pts.append(appender)
#     elif(i >= 75 + 5 and i < 90 + 5):
#         appender += 0.001
#         pts.append(appender)
#     else:
#         pts.append(appender)
#
# print(len(pts))
#
# fig = plt.figure()
# ax = fig.add_subplot(1, 5, 1)
#
# def animate(i):
#     if(i < len(pts)):
#         ax.clear()
#         ax.bar(" ", pts[i], color='brown')
#         ax.set_ylim(0, 1)
#         if(len(str(i)) == 1):
#             fig.savefig('plot-000' + str(i) + '.png')
#         elif(len(str(i)) == 2):
#             fig.savefig('plot-00' + str(i) + '.png')
#         elif(len(str(i)) == 3):
#             fig.savefig('plot-0' + str(i) + '.png')
#         elif(len(str(i)) == 4):
#             fig.savefig('plot-' + str(i) + '.png')
#     else:
#         print("finished")
#
# ani = animation.FuncAnimation(fig, animate, interval=40)


plt.show()
