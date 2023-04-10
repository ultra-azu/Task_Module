This launch files initlialize all MoveIt modules. The core and fundamental launch file is `move_group.launch`.

This file powers on the following modules:

### Planning Functionality:
It activates the planning pipeline with the library OMPL.

### Trajectory execution Functionality
This uses the trajectory_execution.launch.
This sets the controllers parameters and the trajectory execution manager. You can check them in the `config/controllers.yaml` file.

### Sensors Functionality
This calls the sensors_manager.launch file. This file sets the parameters of the octomap_resolution and the max_range. Then it includes the moveit_sensor_manager.launch file. This file sets the parameters of the sensor plugin and the sensor update monitor. You can check them in the `config/sensors_kinect.yaml` file.


### Move Group Node / Action Server
This calls the move_group.launch file. This file sets the parameters of the move group node. You can check them in the `config/move_group.yaml` file.
