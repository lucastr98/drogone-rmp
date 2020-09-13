import numpy as np
import matplotlib.pyplot as plt
import rosbag
import yaml

bags = []
with open(r'evaluation.yaml') as file:
    list = yaml.safe_load(file)
    rosbags = list['rosbags']
    for bag in rosbags:
        bags.append(rosbag.Bag(bag + ".bag"))

starting_times = []
for bag in bags:
    for topic, msg, t in bag.read_messages(topics='/firefly/command/trajectory'):
        starting_times.append(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
        break

print(starting_times)
