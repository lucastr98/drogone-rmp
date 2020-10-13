# DroGone Motion Planning RMP
This repository contains the implementation of the Bachelor Thesis "Motion Planning for DroGone using Riemannian Motion Policies". The following packages were newly implemented, the others were taken to a large extent from DroGone [1]:
* drogone_motion_planner
* drogone_dummy_detector
* drogone_transformation_lib
* drogone_msgs_rmp
This thesis was based on Riemannian Motion Policies, which are presented in [2]. An existing implementation of RMPs [3] was used where minor changes were made in a fork [4].

**Author**: Luca Strässle 

[1]: A. Balestra, M. Baumgartner, J. Becker, I. Boschung, L. Hänsli, S. Laasch,N. Naimi, F. Stadler, S. Steiner, and L. Strässle, “Autonomous detection,tracking and interception of a multicopter,” 2020.

[2]: N. D. Ratliff, J. Issac, and D. Kappler, “Riemannian motion policies”, CoRR, vol. abs/1801.02854, 2018.

[3]: M. Pantic, “rmpcpp,” https://github.com/ethz-asl/rmpcpp, 2020, last accessed 15 May 2020.

[4]: L. Strässle and M. Pantic, “drogone-rmpcpp,” https://github.com/lucastr98/rmpcpp, 2020.

## Dependencies
This Repository depends on some other packages that have to be installed.
```
git clone git@github.com:lucastr98/rmpcpp.git (branch master_luca)
git clone git@github.com:Jonny-air/rotors_simulator.git (branch DroGone)
git clone git@github.com:lucastr98/mav_control_rw.git (branch DroGone)
git clone git@gitlab.ethz.ch:drogone/drogoneautonomy.git
git clone git@github.com:nanaimi/mbzirc_2020_challenge_1.git

git clone git@github.com:catkin/catkin_simple.git
git clone git@github.com:ethz-asl/eigen_catkin.git
git clone git@github.com:ethz-asl/eigen_checks.git
git clone git@github.com:ethz-asl/glog_catkin.git
git clone git@github.com:ethz-asl/mav_comm.git
git clone git@github.com:ethz-asl/mav_tools.git (branch drogone)
git clone git@github.com:ethz-asl/nlopt.git
git clone git@github.com:ethz-asl/yaml_cpp_catkin.git
```
For the world that is active in gazebo right now and the related altitude_node some more packages are necessary.
```
git clone git@github.com:ethz-asl/cad-percept.git (branch feature/altitude_node)
git clone git@github.com:ethz-asl/catkin_boost_python_buildtool.git
git clone git@github.com:ethz-asl/cgal_catkin.git
git clone git@github.com:ethz-asl/json_catkin.git
git clone git@github.com:ethz-asl/minkindr.git
git clone git@github.com:ethz-asl/numpy_eigen.git
```

## Run Simulation
Before starting the simulation a parameter has to be set to indicate the velocity of the target multicopter to the motion planner. It can be set to 0, 2 or 4 meters per second in ".../drogone_rmp/nodes/drogone_motion_planner/cfg/firefly_planner_params.yaml" with the **target_velocity** parameter. Also the parameter **no_catch** can be set. If it is "True" there is no switch to catch mode. If it is "False" the whole autonomy pipeline with catch mode is active.

In ".../drogone_rmp/nodes/drogone_victim_drone/src/victim_drone.py" in line 27 the velocity of the target can be changed. Per default it is set to 2 meters per second. The target velocity can also be changed in the GUI once the simulation is started.

Start the simulation with this command:
```
roslaunch drogone_simulation RMP_sim.launch
```
Then a GUI and RVIZ should open automatically. If the gazebo gui is desired it can be done by changing the 'gui' parameter to true in the launch file ".../drogone_rmp/drogone_simulation/launch/simulationMPC.launch".

Before taking off it one should make sure, that the altitude node is running, which sometimes takes some time to start up. If the following topic is publishing everything is ready to start.
```
rostopic echo /altitude_node/intersection
```
If the world that is used in the gazebo simulation should be displayed in RVIZ this can be done by setting the tick on "TriangleMeshDisplay" in the RVIZ menu.

After Take Off the Gui should look like this:
<img src="/images/GUI_wait_for_autonomous.png">

If nothing is inserted in the boxes at the right and "Change Starting Point" is pressed, the target jumps to a random point. However the starting point can also be changed to a desired position with the three boxes ito the right of the "Change Starting Point" Button. If this is changed, it has to be confirmed by pressing the button. If the position is set like this, a(n) (initial) flight direction of the target has to be defined as well. This can be done with the three boxes "velocity vector" at the upper right. 

For a static try, the "Start Autonomous Flying" button can be pressed to start the carrier multicopter. For special flight manoeuvres the "linear path" or "random path" buttons can be pressed. In this case the carrier multicopter starts automatically. The "circular path" and "eight path" buttons should also work, but have not been tried yet. 