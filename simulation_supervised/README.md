## Simulation-supervised
This part of the simulation-supervised package is the top level.

The **config** directory defines the parameters for a simulated or real(bebop) drone,
as well as the default training/evaluation values like minimum allowed distance and maximum distance
travelled before success.

- define training/evaluation settings (settings might be reset by the launch file of the specific environment)
	- data_param: used when creating a new offline dataset
	- online_param: used for flying online
	- debug_param: used for debuggin
- define topics and messages
	- bebop_real: used when flying with a real bebop drone by bebop autonomy (https://github.com/AutonomyLab/bebop_autonomy)
	- sim_drone: used when flying with the simulated quadcopter (https://github.com/kkelchte/hector_quadrotor)
- controller-settings:
	- ps3
	- bigben


The **scripts** directory has the 3 most important scripts for using this package:
- train_model
- evaluat_model
- train_and_evaluate_model

The scripts start python with the following two scripts:
- start_python 
- start_python_docker

The other two scripts are extra:
- create_data_real: used for creating a new almost_collision_dataset


## Example usage:

### From laptop /home/klaas
Ensure the environment is setup correctly:
source /opt/ros/kinetic/setup.bash
source /home/klaas/drone_ws/devel/setup.bash --extend
source /home/klaas/simsup_ws/devel/setup.bash --extend

$ roscd simulation_supervised
$ ./scripts/evaluate_model.sh -s start_python.sh -m auxd -t testing_on_laptop

### From docker with graphics
$ sudo nvidia-docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/klaas:/home/klaas -u klaas kkelchte/ros_gazebo_tensorflow
$ export DISPLAY=:0
$ export LD_LIBRARY_PATH=/usr/local/nvidia/lib64:$LD_LIBRARY_PATH
$ roscd simulation_supervised
$ ./scripts/evaluate_model.sh -s start_python.sh -m auxd -t testing_on_laptop
