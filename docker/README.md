In this folder we have defined all the dockerfiles to run the Submarine code.


TODO: Divided command in two. Development and Production. Development will run the code of the submarine in the uuv_simulator andthe other one will not have the dependencies for the simulator, and will remove any additional dependencies used for the development. Example will be to use the the more basic ros dependencies for running the submarine.



Because we have different nodes that uses diferrent distributions of ROS we have to use Docker Networks. The main problem that solves using Docker Networks is that Melodic uses Ubuntu 18 and Noetic Ubuntu 20 meaning that we can't have a single Docker container with both Melodic and Noetic.



## How to run the Docker Network


```bash
docker network create ros-network
docker run --network=ros-network --name ros-melodic-container -it [your-melodic-image]
docker run --network=ros-network --name ros-noetic-container -it [your-noetic-image]

```


