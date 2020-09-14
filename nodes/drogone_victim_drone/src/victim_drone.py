#!/usr/bin/env python
#Author: Felix Stadler, 15.2.20
import rospy
import roslib
# import time
import numpy as np
import math
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray #for communication between GUI and Victim Drone.py
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Transform, Twist, PoseStamped, Vector3, PoseArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint



class VictimDrone(object):

    def __init__(self,name):

        # calculate variables for randomization for evaluation
        self.phi_eval = np.random.uniform(0, 2 * np.pi)
        self.theta_eval = np.random.uniform(-np.pi / 6, np.pi / 6)
        self.z_C_eval = np.random.uniform(5, 15)

        #parameters of the trajectory/path, params with ** at the end are changeable parameters (most of them are also changeable in the GUI)
        #--------------------------------------------------------help variables--------------------------------------------------
        self.start_moving = 0
        self.initialize = 1

        #--------------------------------------------------------basic variables--------------------------------------------------
        self.velocity = 2.0 #** #velocity of victim drone
        self.velocity_fix = self.velocity #for velocity variation
        self.velocity_spread = 0 #0.08 #** #velocity variation. set to zero if no velocity change is wanted
        self.velocity_max = 5 #maximum velocity for varaition
        self.velocity_min = 0.25 #maximum velocity for varaition
        self.rate = 100.0 #** #rate in Hz at which /victim_drone/odometry is published
        self.dt = 1/self.rate #time interval between two published msgs
        self.dt_dur= rospy.Duration(self.dt)
        #starting point
        self.starting_point = Point()
        self.starting_point.x = 0.0 #**
        self.starting_point.y = 0.0 #**
        self.starting_point.z = 15.0 #**

        #------------------------------------linear path---------------------------------------------
        #vector of linear moving victim drone, normalized
        self.motion_vector = Vector3()
        self.motion_vector.x = 1/math.sqrt(2) #**
        self.motion_vector.y = 1/math.sqrt(2) #**
        self.motion_vector.z = 0 #**


        #------------------------------------circular path---------------------------------------------
        self.phi_circle = -np.pi #starting angle for circle path
        self.radius = 10 #** #ALSO A PARAMETER OF EIGHT PATH
        self.theta = np.pi/36 #** #theta is angle curve is turned around y-axis  #ALSO A PARAMETER OF EIGHT PATH

        #------------------------------------eight path---------------------------------------------
        self.distance = 38 #** #distance between the two half circles for diagonal eight, d>2r
        self.phi_0 = np.arccos(2.0*self.radius/(self.distance))  #starting angle for eight path
        self.phi = -2 * np.pi
        self.s = -self.velocity * self.dt #starting diagonal self.distance
        self.x = 0
        # RADIUS AND THETA ANGLE ARE AT THE CIRCULAR PATH PARAMS SECTION
        self.vector = Vector3() # for diagonal part of eight
        self.circle = 2
        self.diagonal = 1

        #------------------------------------random path---------------------------------------------
        #parameters of random path, smooth path 0.5 and 0.8, aggressive 1.0 amd 1.5
        self.theta_spread = self.velocity * 0.5 #** #half of the angle interval width in degrees for new angle/ smooth path 0.5, aggressive 1.0
        self.phi_spread = self.velocity * 1.5 #** #half of the angle interval width in degrees for new angle/ smooth path 0.8, aggressive 1.5
        self.stay_inside = 1 #** #set to zero, if random path should also be continued outside the arena (make sure that theta and phi spread have same value)
        self.smooth = 1 #**  #when drone leaves the frame, should return path be smooth (=1)or more aggressive (=0)
        #if drone should stay inside certain frame (stay inside == 1), default values for rviz arena frame are x=35, y=20, z_max=40, z_min=6
        self.arena_x = 35 #** # limit in x direction, symmetric so from -arena_x to arena_x (in m)
        self.arena_y = 20 #** #symmetric (in m)
        self.arena_z_max = 40 #** #upper z limit (in m)
        self.arena_z_min = 6 #** #lower z limit (in m)
        # will be one, if arena frame outside (x,y) or above/below (z) is reached (random path)
        self.outside_arena = 0
        self.above_arena = 0
        self.below_arena = 0

        #------------------------------------drone caught---------------------------------------------
        self.caught = 0
        self.offset = Point() #if drone caught, save offset between the two odometries
        self.drogone_odom = Odometry()

        #-----------------------------------plan path/ dummy tracker params---------------------------------------------
        self.dummy_tracker = 1 #set to zero if dummy tracker should not be used
        self.time_difference_max = rospy.Duration(2.1) #** in seconds # max time difference between furthest point dummy tracker sends and time now
        self.time_interval = 0.1 #** in seconds #time interval between two points in the MultiDOFJointTrajectory msgs.
        self.time_interval_dur = rospy.Duration(self.time_interval)
        self.pub_rate = 10.0 #** #rate in Hz at which /ground_truth_tracker is published
        self.pub_rate_dur = rospy.Duration(1/self.pub_rate)
        self.delete_traj = 1 # delete array of points from victim drone trajectory

        sub_GUI = rospy.Subscriber("GUI/victim_drone_param", Float32MultiArray, self.GUI_callback, queue_size=100)

    def plan_path(self):

        pub_odometry =rospy.Publisher("victim_drone/odometry", Odometry, queue_size=100)
        pub_tracker =rospy.Publisher("drogone/ground_truth_tracker", MultiDOFJointTrajectory, queue_size=100)
        offset_sub = rospy.Subscriber("/visualization/victim_offset",  Point, self.offset_callback, queue_size=10)
        drogone_odom_sub = rospy.Subscriber("/peregrine/odometry_sensor1/odometry",  Odometry, self.drogone_odom_callback, queue_size=100)

        #store the whole path in this variable
        victim_path = Path()
        #store the current twist here
        twist = Twist()
        length = 0 #length of path array

        #help variables for dummy tracker
        timer_pub_tracker = rospy.Time.now()
        length_of_victim_traj_points = int(self.time_difference_max / self.time_interval_dur) +1 #add one to make sure to have enough points (more than 2 secs in future)
        number_of_poses_inbetween = int(self.time_interval/self.dt) #number of pose elements in path array which are not copied to victim_traj
        first_msg_info = 1

        publishing_rate = rospy.Rate(self.rate)
        while (not rospy.is_shutdown()):

            if self.delete_traj:
                del victim_path.poses[1:length]
                self.delete_traj = 0

            # calculate length of path array
            length = len(victim_path.poses)
            # calculate velocity
            if self.velocity > self.velocity_max:
                self.velocity += np.random.uniform(-self.velocity_spread, 0) * self.velocity_fix
            elif self.velocity < self.velocity_min:
                self.velocity += np.random.uniform(0, self.velocity_spread) * self.velocity_fix
            else:
                self.velocity += np.random.uniform(-self.velocity_spread, self.velocity_spread) * self.velocity_fix

            #calculate new segment of path
            if self.initialize:
                new_pose = self.starting_position()

            elif self.caught:
                #if drone caught dummy tracker does not publish anything anymore
                victim_odom = self.drone_caught(victim_path.poses[length-1])
                victim_odom.header.stamp = rospy.Time.now()
                victim_odom.header.frame_id = victim_path.poses[length-1].header.frame_id
                #publish and sleep
                pub_odometry.publish(victim_odom)
                publishing_rate.sleep()
                continue

            elif self.start_moving == 0:
                #hovering -> pose does not change
                new_pose = PoseStamped()
                new_pose.pose = victim_path.poses[length-1].pose

            elif self.start_moving == 1:
                new_pose = self.linear_path(victim_path.poses[length-1])

            elif self.start_moving == 2:
                new_pose = self.circular_path()

            elif self.start_moving == 3:
                new_pose = self.diagonal_eight_path(victim_path.poses[length-1])

            elif self.start_moving == 4:
                new_pose, twist = self.random_path(victim_path.poses[length-1], twist)

            #add pose to path and header
            victim_path.poses.append(new_pose)
            if length == 0:
                victim_path.poses[0].header.stamp = rospy.Time.now()
                victim_path.poses[0].header.frame_id = "world"
                continue
            else:
                victim_path.poses[length].header.frame_id = victim_path.poses[length-1].header.frame_id
                victim_path.poses[length].header.stamp = victim_path.poses[length-1].header.stamp + self.dt_dur

                # check if path array is full with all the poses for the duration of *time_difference_max  +  self.time_interval_dur (to make sure last point is more than 2 secs in the future)* and one extra point for velocity calculations at the end
                if victim_path.poses[length-1].header.stamp - victim_path.poses[0].header.stamp >= self.time_difference_max +  self.time_interval_dur:

                    #only publish /ground_truth_tracker at certain time intervals
                    current_time = rospy.Time.now()
                    if self.dummy_tracker and (current_time - timer_pub_tracker >= self.pub_rate_dur):
                        victim_traj = MultiDOFJointTrajectory()
                        victim_traj.header.frame_id = victim_path.poses[0].header.frame_id

                        #fill victim_traj with future points of victim drone
                        for i in range(0, length_of_victim_traj_points):
                            #get pose from path
                            get_pose = victim_path.poses[i*number_of_poses_inbetween] #type PoseStamped

                            victim_transform = Transform()
                            victim_transform.translation.x = get_pose.pose.position.x
                            victim_transform.translation.y = get_pose.pose.position.y
                            victim_transform.translation.z = get_pose.pose.position.z
                            victim_traj_point = MultiDOFJointTrajectoryPoint()
                            victim_traj_point.transforms.append(victim_transform)

                            #calcuate velocity for last point in array
                            if (i+1)*number_of_poses_inbetween >= len(victim_path.poses) or \
                               (i+1) == length_of_victim_traj_points:

                                victim_twist = self.calculate_velocity(victim_path.poses[(i*number_of_poses_inbetween)+1], get_pose)
                                victim_traj_point.velocities.append(victim_twist)
                                victim_traj.points.append(victim_traj_point)
                                #check if Dummy tracker has all the points
                                if not i+1 == length_of_victim_traj_points:
                                    rospy.logwarn("VICTIM DRONE: Dummy tracker msg has length %s and not expected length %s", len(victim_traj.points), length_of_victim_traj_points)
                                break

                            victim_traj.points.append(victim_traj_point)

                        #add time_from_start for each Point
                        # time_from_start = rospy.Time.now()
                        time_from_start = rospy.Time(0)
                        for i in range(0, len(victim_traj.points)):
                            victim_traj.points[i].time_from_start = time_from_start
                            time_from_start  +=  self.time_interval_dur

                        # victim_traj.points[0].time_from_start = rospy.Time.now()
                        victim_traj.header.stamp = rospy.Time.now()
                        pub_tracker.publish(victim_traj)

                        #log info about dummy tracker at the beginning
                        if first_msg_info:
                            rospy.logwarn("VICTIM DRONE: Dummy tracker MultiDOFJointTrajectory Msg info: \n \
                            Number of MultiDOFJointTrajectoryPoints in Msgs: %s \n \
                            Time interval between 2 MultiDOFJointTrajectoryPoints: %s secs \n \
                            Time difference between first and last MultiDOFJointTrajectoryPoint: %s secs \n \
                            Msgs publishing rate: %s Hz", length_of_victim_traj_points, self.time_interval, self.time_interval*(length_of_victim_traj_points-1), self.pub_rate)
                            first_msg_info = 0

                        #reset timer
                        timer_pub_tracker = rospy.Time.now()

                    # get current pose to publish for rviz
                    victim_odom = Odometry()
                    victim_odom.header.frame_id = victim_path.poses[0].header.frame_id
                    victim_odom.pose.pose = victim_path.poses[0].pose

                    # publish and sleep
                    victim_odom.header.stamp = rospy.Time.now()
                    pub_odometry.publish(victim_odom)

                    #remove the current segment from the path array
                    del victim_path.poses[0]

                    publishing_rate.sleep()

                else:
                    #continue until array is filled up so dummy tracker can publish the msg
                    continue

    def starting_position(self):
        victim_pose = PoseStamped()
        victim_pose.pose.position.x = self.starting_point.x
        victim_pose.pose.position.y = self.starting_point.y
        victim_pose.pose.position.z = self.starting_point.z
        self.initialize = 0
        self.caught = 0
        self.start_moving = 0
        return victim_pose

    def linear_path(self, current_pose):
        #distance covered in certain time interval
        ds = self.velocity * self.dt

        victim_pose = PoseStamped()
        victim_pose.pose.position.x = current_pose.pose.position.x + self.motion_vector.x * ds
        victim_pose.pose.position.y = current_pose.pose.position.y + self.motion_vector.y * ds
        victim_pose.pose.position.z = current_pose.pose.position.z + self.motion_vector.z * ds
        return victim_pose

    def circular_path(self):
        #midpoints of the circle
        x0 = self.starting_point.x - self.radius
        y0 = self.starting_point.y
        z0 = self.starting_point.z + self.radius * math.sin(self.theta)

        #distance covered in certain time interval
        ds = self.velocity * self.dt
        # angle difference covered in certain time interval
        dphi = ds/self.radius

        victim_pose = PoseStamped()
        victim_pose.pose.position.x = x0 - self.radius * math.cos(self.theta) * math.cos(self.phi_circle)
        victim_pose.pose.position.y = y0 + self.radius * math.cos(self.theta) * math.sin(self.phi_circle)
        victim_pose.pose.position.z = z0 + self.radius * math.sin(self.theta) * math.cos(self.phi_circle)
        self.phi_circle += dphi

        return victim_pose

    def diagonal_eight_path(self, current_pose):

        #distance covered in certain time interval
        ds = self.velocity * self.dt
        # angle difference covered in certain time interval
        dphi = ds/self.radius

        #midpoints of the two circles
        x1 = self.starting_point.x + self.distance/2
        x2 = self.starting_point.x - self.distance/2
        y1 = self.starting_point.y
        y2 = self.starting_point.y
        z1 = self.starting_point.z - (self.distance/2) * math.sin(self.theta)
        z2 = self.starting_point.z + (self.distance/2) * math.sin(self.theta)

        if self.circle == 1 and self.phi >= (4*np.pi/2 - self.phi_0) and self.phi < (4*np.pi/2 - self.phi_0 + dphi):
            self.phi = 4*np.pi/2 - self.phi_0

        elif self.circle == 1 and self.phi > 4*np.pi/2 - self.phi_0:
            self.circle = 2
            self.diagonal = 1
            self.phi = np.pi - self.phi_0
            a = x2 - self.radius * math.cos(self.theta) * math.cos(self.phi) - current_pose.pose.position.x
            b = y2 + self.radius * math.cos(self.theta) * math.sin(self.phi) - current_pose.pose.position.y
            c = z2 + self.radius * math.sin(self.theta) * math.cos(self.phi) - current_pose.pose.position.z
            self.x = math.sqrt(a*a + b*b + c*c)
            self.vector.x = a/ self.x
            self.vector.y = b/ self.x
            self.vector.z = c/ self.x

        elif self.circle == 2 and self.phi <= - 2*np.pi/2 + self.phi_0 and self.phi > - 2*np.pi/2 + self.phi_0 - dphi:
            self.phi = - 2*np.pi/2 + self.phi_0

        elif self.circle == 2 and self.phi <= - 2*np.pi/2 + self.phi_0:
            self.circle = 1
            self.diagonal = 1
            self.phi = self.phi_0
            a = x1 - self.radius * math.cos(self.theta) * math.cos(self.phi) - current_pose.pose.position.x
            b = y1 + self.radius * math.cos(self.theta) * math.sin(self.phi) - current_pose.pose.position.y
            c = z1 + self.radius * math.sin(self.theta) * math.cos(self.phi) - current_pose.pose.position.z
            self.x = math.sqrt(a*a + b*b + c*c)
            self.vector.x = a/ self.x
            self.vector.y = b/ self.x
            self.vector.z = c/ self.x

        elif self.s>(self.x-ds):
            self.diagonal = 0
            self.s=0

        victim_pose = PoseStamped()
        if self.diagonal == 1:
            #calculating position
            victim_pose.pose.position.x = current_pose.pose.position.x + self.vector.x * ds
            victim_pose.pose.position.y = current_pose.pose.position.y + self.vector.y * ds
            victim_pose.pose.position.z = current_pose.pose.position.z + self.vector.z * ds
            self.s += ds

        elif self.circle == 1:
            #calculating position
            victim_pose.pose.position.x = x1 - self.radius * math.cos(self.theta) * math.cos(self.phi)
            victim_pose.pose.position.y = y1 + self.radius * math.cos(self.theta) * math.sin(self.phi)
            victim_pose.pose.position.z = z1 + self.radius * math.sin(self.theta) * math.cos(self.phi)
            self.phi += dphi

        elif self.circle == 2:
            #calculating position
            victim_pose.pose.position.x = x2 - self.radius * math.cos(self.theta) * math.cos(self.phi)
            victim_pose.pose.position.y = y2 + self.radius * math.cos(self.theta) * math.sin(self.phi)
            victim_pose.pose.position.z = z2 + self.radius * math.sin(self.theta) * math.cos(self.phi)
            self.phi -= dphi

        return victim_pose

    def random_path(self, current_pose, twist):

        #random path planning
        #randomly select a point on the surface of a sphere with radius ds (distance covered in time interval between two published msgs)
        #point can only be selected in a certain area of the whole sphere, depending on the theta and phi spread (spherical coordiantes used)
        #referenced point on the sphere is where the current velocity vector intersects the sphere
        #the reference point is the centre of the certain surface area and the agle spread start from there

        #distance covered in certain time interval
        ds = self.velocity * self.dt

        # transfer victim velocity vector from kartesian x,y,z into spherical coordinates r, phi, theta
        r = np.sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y + twist.linear.z*twist.linear.z)
        # if velocity == 0 take the motion vector as startign vector
        if r == 0:
            twist.linear.x = self.motion_vector.x
            twist.linear.y = self.motion_vector.y
            twist.linear.z = self.motion_vector.z
            r = 1 # as motion vector is of magnitude 1
        theta_old = math.acos(twist.linear.z/r)
        phi_old = math.atan2(twist.linear.y, twist.linear.x)


        #calculate new angles with randomly picked angle difference
        if self.outside_arena == 0 and self.above_arena == 0 and self.below_arena == 0:
            theta_new = np.random.uniform(-self.theta_spread, self.theta_spread) * np.pi/180 + theta_old
            phi_new = np.random.uniform(- self.phi_spread, self.phi_spread) * np.pi/180 + phi_old

        #If drone moved out of the frame, changes the spread of the angle such that the drone moves back into the frame
        #below limit
        elif self.below_arena == 1:
            if self.smooth == 1:
                theta_new = np.random.uniform(0, -self.theta_spread) * np.pi/180 + theta_old
            else:
                theta_new = np.random.uniform(0, -4*self.theta_spread) * np.pi/180 + theta_old
            phi_new = phi_old
        # on edge of arena -> x,y direction
        elif self.outside_arena == 1:
            theta_new = theta_old
            if self.smooth == 1:
                phi_new = np.random.uniform(0, self.phi_spread) * np.pi/180 + phi_old
            else:
                phi_new = np.random.uniform(0, 3*self.phi_spread) * np.pi/180 + phi_old
        # on top of arena -> z direction
        elif self.above_arena == 1:
            if self.smooth == 1:
                theta_new = np.random.uniform(0, self.theta_spread) * np.pi/180 + theta_old
            else:
                theta_new = np.random.uniform(0, 4*self.theta_spread) * np.pi/180 + theta_old
            phi_new = phi_old


        # if stay inside is on, make sure path stays inside a certain arena frame
        if self.stay_inside == 1:
            #check if victim drone is outside the arena and the velocity vector is not pointing towards the centre of the arena
            if (abs(current_pose.pose.position.x + ds * math.sin(theta_new) * math.cos(phi_new)) > self.arena_x or \
               abs(current_pose.pose.position.y + ds * math.sin(theta_new) * math.sin(phi_new)) > self.arena_y) and \
               (current_pose.pose.position.x*twist.linear.x > 0 or current_pose.pose.position.y*twist.linear.y > 0):
                self.outside_arena = 1
            else:
                self.outside_arena = 0
            if (current_pose.pose.position.z + ds * math.cos(theta_new)) > self.arena_z_max and current_pose.pose.position.z*twist.linear.z > 0:
                self.above_arena = 1
            else:
                self.above_arena = 0

        #always make sure victim drone stays above the ground
        if (current_pose.pose.position.z + ds * math.cos(theta_new)) < self.arena_z_min and current_pose.pose.position.z*twist.linear.z < 0:
            self.below_arena = 1
        else:
            self.below_arena = 0

        #calculate new position by transfroming spherical coordinates back to cartesian coordinates
        victim_pose = PoseStamped()
        victim_pose.pose.position.x = current_pose.pose.position.x + ds * math.sin(theta_new) * math.cos(phi_new)
        victim_pose.pose.position.y = current_pose.pose.position.y + ds * math.sin(theta_new) * math.sin(phi_new)
        victim_pose.pose.position.z = current_pose.pose.position.z #+ ds * math.cos(theta_new)

        #calculate new velocity by transfroming spherical coordinates back to cartesian coordinates
        victim_twist = Twist()
        victim_twist.linear.x =  self.velocity * math.sin(theta_new) * math.cos(phi_new)
        victim_twist.linear.y =  self.velocity * math.sin(theta_new) * math.sin(phi_new)
        victim_twist.linear.z =  self.velocity * math.cos(theta_new)

        return victim_pose, victim_twist

    def drone_caught(self, current_pose):

        victim_odom = Odometry()
        # victim_odom = self.drogone_odom
        victim_odom.pose.pose.position.x = self.drogone_odom.pose.pose.position.x + self.offset.x
        victim_odom.pose.pose.position.y = self.drogone_odom.pose.pose.position.y + self.offset.y
        victim_odom.pose.pose.position.z = self.drogone_odom.pose.pose.position.z + self.offset.z

        return victim_odom

    def calculate_velocity(self, new_pose, old_pose):
        victim_twist = Twist()
        dx = new_pose.pose.position.x - old_pose.pose.position.x
        dy = new_pose.pose.position.y - old_pose.pose.position.y
        dz = new_pose.pose.position.z - old_pose.pose.position.z
        magnitude = math.sqrt(dx*dx + dy*dy + dz*dz)
        if (magnitude == 0):
            victim_twist.linear.x = 0
            victim_twist.linear.y = 0
            victim_twist.linear.z = 0
        else:
            victim_twist.linear.x = dx/magnitude * self.velocity
            victim_twist.linear.y = dy/magnitude * self.velocity
            victim_twist.linear.z = dz/magnitude * self.velocity

        return victim_twist

    def GUI_callback(self, data):
        # ---------------------------------------------------VICTIM DRONE parameters --------------------------------------------------------------------------------------------------------------
        # parameters are stored in an array with 16 entires (size_of_array):
        # [0] = type of movement: hovering = 0; linear path = 1; circle = 2; eight = 3; random = 4;
        # [1] = velocity
        # [2] = change starting point, true/false
        # [3] = new x starting point
        # [4] = new y starting point
        # [5] = new z starting point
        # [6] = change motion vector for linear moving path, true/false
        # [7] = new x component motion vector for linear moving path
        # [8] = new y component motion vector for linear moving path
        # [9] = new z component motion vector for linear moving path
        # [10] = restart -> move victim drone back to starting position and hover
        # [11] = turn dummy tracker on
        # [12] = drone caught, true/flase
        # [13] = new radius of circle/eight
        # [14] = mew theta angle tilt in radian
        # [15] = new distance between circles for eight
        # [16] = theta angle width for random path
        # [17] = phi angle width for random path

        # restart option
        if data.data[10] == 1.0:
            self.initialize = 1
            self.delete_traj = 1

        #turn dummy tracker on or off
        if data.data[11] == 1.0:
            self.dummy_tracker = 1
            rospy.logwarn("VICTIM DRONE: Dummy tracker starts publishing")
        elif data.data[11] == 2.0:
            self.dummy_tracker = 0
            rospy.logwarn("VICTIM DRONE: Dummy tracker stops publishing")

        # Drone caught
        elif data.data[12] == 1.0:
            self.caught = 1
            self.calculate_offset = 1
            self.delete_traj = 1

        #take moving path
        else:
            self.start_moving = data.data[0]
            self.delete_traj = 1

        #radius
        if not data.data[13] == 0.0:
            self.radius = data.data[13]
            self.delete_traj = 1
        #tilt
        if not data.data[14] == 0.0:
            self.theta = data.data[14] * np.pi/180
            self.delete_traj = 1
        #distance
        if not data.data[15] == 0.0:
            self.distance = data.data[15]
            self.initialize = 1
            self.delete_traj = 1
        #velocity
        if not data.data[1] == 0.0:
            # self.velocity = data.data[1]
            self.velocity = 2

        #angle spread
        if not data.data[16] == 0.0:
            self.theta_spread = data.data[16]
        if not data.data[17] == 0.0:
            self.phi_spread = data.data[17]

        # starting point parameters
        if data.data[2] == 1.0:
            self.starting_point.x = data.data[3]
            self.starting_point.y = data.data[4]
            self.starting_point.z = data.data[5]
            self.start_moving = data.data[0]
            self.initialize = 1
            self.caught = 0
            self.delete_traj = 1

            # for evaluation
            self.phi_eval = 0
            self.theta_eval = np.pi / 2
            self.z_C_eval = 7
            # self.phi_eval = np.random.uniform(0, 2 * np.pi)
            # self.theta_eval = np.random.uniform(np.pi / 3, np.pi / 3 * 2)
            # self.z_C_eval = np.random.uniform(5, 15)
            init_px_eval = 400
            u_eval = np.cos(self.phi_eval) * init_px_eval
            v_eval = np.sin(self.phi_eval) * init_px_eval
            x_C_eval = u_eval / 1140 * self.z_C_eval
            y_C_eval = v_eval / 1140 * self.z_C_eval
            self.starting_point.x = x_C_eval
            self.starting_point.y = y_C_eval
            self.starting_point.z = self.z_C_eval + 10.0

        # vector of linear moving victim drone parameters
        elif data.data[6] == 1.0:
            direction_x_eval = np.sin(self.theta_eval) * np.cos(self.phi_eval)
            direction_y_eval = np.sin(self.theta_eval) * np.sin(self.phi_eval)
            direction_z_eval = np.cos(self.theta_eval)
            magnitude = np.sqrt(direction_x_eval*direction_x_eval + direction_y_eval*direction_y_eval + direction_z_eval*direction_z_eval)
            self.motion_vector.x = direction_x_eval / magnitude
            self.motion_vector.y = direction_y_eval / magnitude
            self.motion_vector.z = direction_z_eval / magnitude

            #normalize vector
            # magnitude = np.sqrt(data.data[7]*data.data[7] + data.data[8]*data.data[8] + data.data[9]*data.data[9])
            # self.motion_vector.x = data.data[7]/ magnitude
            # self.motion_vector.y = data.data[8]/ magnitude
            # self.motion_vector.z = data.data[9]/ magnitude
            self.delete_traj = 1


    def offset_callback(self, data):
        #offset between victim and drogone drone if drone caught, - as offset is calclated as drogone.pos - victim.pos
        self.offset.x = - data.x
        self.offset.y = - data.y
        self.offset.z = - data.z

    def drogone_odom_callback(self, data):
        #save drogone odometry
        self.drogone_odom = data

if __name__ == '__main__':

    # #wait 3 seconds till everyhthing starts up
    # rospy.sleep(5.0)

    rospy.init_node('Victim_Drone')
    rospy.logwarn('VICTIM DRONE: VICTIM DRONE IS READY')

    #initialize class
    VictimDrone = VictimDrone("victim")
    VictimDrone.plan_path()

    rospy.spin() #rospyspin keeps node running
