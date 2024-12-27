# ROS Workspace

To setup the workspace, you can either use compile directly on your computer or quickly set up on [Docker](https://www.docker.com/).

## Compiling directly

1. Ensure that you are running Ubuntu 24.04 (Noble Numbat) on your computer or in a virtualized environment

2. Install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)

3. Install necessary dependencies for our ROS2 workspace. You can look through the Dockerfile (`Software_2025/ros_worksapce/Dockerfile`) for the most up-to-date list of dependancies.

4. Run `colcon build` in `Software_2025/ros_workspace`

5. Source the workspace by running `source setup.bash` in `Software_2025/ros_workspace/install`

To run our C++ frontend (which is a standalone alternative to the Web Frontend)
```
ros2 run cpp_frontend cpp_frontend_node
```

Choose the backend launch file to run based on the ROV (ex. Waterwitch or Beaumont) and whether or not you are using the [simulation_environment](https://github.com/EasternEdgeRobotics/rov-sim). Once chosen, you can run:
```
ros2 launch <backend package (ex. beaumont_backend)> <launch file <ex. simulation_beaumont_startup>>
```
If the launch file run starts an instance of ROSBridge Server, you can control the ROV using the [Web Frontend](../web_frontend/)

If you make changes and would like to recompile, restart from step 4.

## Setting up on Docker

1. Install [Docker](https://www.docker.com/) and ensure you can use `docker compose`
 
2. Run the following within a terminal in `Software_2025/`.
```
sudo docker compose -f dev_compose.yaml up 
```

Note that `compose.yaml` is intended to run on the real ROVs' onboard Raspberry Pi computers and gives them access to the i2c bus for board communication. `dev_compose.yaml` runs the same code, but for interfacing with our [simulation environment](https://github.com/EasternEdgeRobotics/rov-sim). 

You should now have two docker containers, `web_frontend` and `eer_ros_packages`. 

You can type "localhost" in your browser to view the web frontend or type `sudo docker exec -it eer_ros_packages bash` in a terminal to interact with our ROS2 nodes.

If you make changes and would like to recompile:
```
sudo docker compose -f dev_compose.yaml up ---build <web_frontend and/or eer_ros_packages>
```