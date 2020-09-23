import cv2
import numpy as np

img_array = []
for i in range(1050):
    # if i % 4 == 0:
    #     continue
    if(len(str(i)) == 1):
        name = '000' + str(i)
    elif(len(str(i)) == 2):
        name = '00' + str(i)
    elif(len(str(i)) == 3):
        name = '0' + str(i)
    elif(len(str(i)) == 4):
        name = str(i)
    img = cv2.imread('/home/severin/luca_ws/rosbags/distance2ground_evaluation/video/plot-' + name + '.png')
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)

out = cv2.VideoWriter('d2g_plot.mp4',cv2.VideoWriter_fourcc(*'MP4V'), 100, size)
# out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 75, size)

for i in range(len(img_array)):
    out.write(img_array[i])
out.release()
