# simulation-supervised

The simulation-supervised package combines different sets of code needed to train a DNN policy to fly a drone.

## Dependencies
* [online_training](https://www.github.com/kkelchte/pilot_online): the tensorflow code used for training and running the DNN policy.
* [drone_simulator](https://www.github.com/kkelchte/hector_quadrotor): a simulated drone model for Gazebo. This is a copy of the original [hector quadrotor model](http://wiki.ros.org/hector_quadrotor).
OR
* [bebop_autonomy](https://github.com/kkelchte/bebop_autonomy): a copy of the bebop autonomy package of ROS. This package allows you to test the DNN in the real-world. The copy is only for ensuring stability while performing research. There are no significant modifications so it is probably best to use [the original](https://github.com/AutonomyLab/bebop_autonomy). If you are using the Doshico docker image, it is already installed globally.

## Installation
This package is best build in a separate [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
```bash
mkdir -p ~/simsup_ws/src
cd ~/simsup_ws && catkin_make
cd ~/simsup_ws/src
git clone https://github.com/kkelchte/simulation_supervised
cd ~/simsup_ws && catkin_make
```

You will have to set the correct path to your tensorflow pilot_online package.

In case you are using different drone models, you will have to adjust the [config.yaml](https://github.com/kkelchte/simulation-supervised/blob/master/simulation_supervised/config/sim_drone.yaml) file in order to set the correct rosparams.

## Run some experiments

Here are some common used setting combinations in order to remember them:
```
# Test online the performance of a heuristic defined in tensorflow/gt_depth_online/../rosinterface.py using for instance recovery cameras flying 3 times through a generated canyon
$ ./scripts/evaluate_model.sh -m auxd -s start_python_sing_gtdepth.sh -t test_depth_online -r true -w canyon -n 3
# Train online in canyon, forest, sandbox
$ ./scripts/train_model.sh -m mobilenet_025

```