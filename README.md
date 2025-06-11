# Racing-CARLA
An open-source simualtor for autonomous racing built upon [CARLA](https://github.com/carla-simulator/carla).

## Setup
R-CARLA heavily depends on [CARLA](https://github.com/carla-simulator/carla) and the [CARLA ROS Bridge](https://github.com/carla-simulator/ros-bridge.git) so you first need to set this up:

### CARLA
For further instructions on how to setup CARLA please refer to the setup guide in the CARLA repository. R-CARLA currently uses the UE4 version of CARLA.
```
git clone https://github.com/carla-simulator/carla.git -b dev
./Update.sh
```

You can then download the R-CARLA [assets](https://drive.google.com/file/d/1wUQL6IQm_-O-gAnQn7j2ZK2K7GP3NjZe/view?usp=sharing) and unzip them to `~/carla/Unreal/CarlaUE4/Content/Carla/`. You can then replace the files `Unreal/CarlaUE4/Config/DefaultGame.ini`, `Unreal/CarlaUE4/Config/DefaultEngine.ini` and `Unreal/CarlaUE4/Content/Carla/Blueprints/Vehicles/VehiclesFactory.uasset` with THIS AND THIS AND THIS. 

Finally you can run `make package` to build a packaged version with the new assets. If you want to add your custom maps and vehicles you can follow THIS tutorial.

### ROS-Bridge
We are using the ros-bridge with docker so make sure that docker is installed.
Before you build the bridge you need to add the definition of your vehicle to `ros-bridge/carla_spawn_objects/config/objects.json`. If you are using the F1TENTH car from the R-CARLA assets you can use THIS file.

```
git clone https://github.com/maubrunn/ros-bridge.git -b r-carla
cd ros-bridge
git submodule update --init --recursive
cd docker
```
You can now build for ROS1 `./build.sh -r noetic -c YOUR_CARLA_VERSION` or ROS2 `./build.sh -r foxy -c YOUR_CARLA_VERSION`.

### R-CARLA
Now we can finally build R-CARLA. 
```
docker compose build ros1 OR ros2
```

## Usage
First launch CARLA and then the ros-bridge:
```
cd ros-bridge/docker
./run.sh -t foxy ros2 launch carla_ros_bridge carla_ros_bridge.launch.py synchronous_mode:=False fixed_delta_seconds:=0.0 town:=gokart_map timeout:=10
./run.sh -t foxy roslaunch carla_spawn_objects carla_spawn_objects.launch.py
```

Now we can run R-CARLA `docker compose up ros1 OR ros2`

This launches all interfaces of R-CARLA, they can be configured in `cfg/config.yaml`

### Config

| Name      | Description |
| ----------- | ----------- |
| env/use_joystick      | Defines if a joystick is used to control the car  |
| env/convert_pc_to_scan| Defines if pointclouds should be converted to laserscan msgs (for 2D lidars)|
| traffic_interface/num_npc        | Number of spawned npcs |
| traffic_interface/map_name        | The used map |
| traffic_interface/map_name        | The used map |



### Traffic Interface
This spawnes multiple NPC's and moves them along a predefined trajectory. The trajectory is taken from the `global_waypoints.json` file in `interfaces/traffic_interface/maps/MAP_NAME`. The file needs to have the same structure as defined in [here](https://github.com/TUMFTM/global_racetrajectory_optimization).

For each NPC a random spawn point and velocity scaler is chosen.

### Drive Interface
The drive interface moves the ego vehicle in CARLA. At the moment a dynamic single-track model is used. If you want to use custom dynamics this can be done easily. The drive interface calls two services `/rcarla/physics_interface/init` to set an initial position and `/rcarla/physics_interface/step` for each update step. You can add these two services to your custom physics engine and it will be used to update the position of the ego vehicle in CARLA. The [physics interface](interfaces/drive_interface/src/drive_interface/physics_interface.py) should give you an understanding on how to integrate it.

### Sensors Interface
Because we bypass the CARLA dynamics the IMU and wheel odom simualtion do not work anymore. These are done in the sensors interface. If you add your custom physics they will not work anymore. In your callback for each step update you need to add a service call to `/rcarla/sensors_interface/odom_update` and `/rcarla/sensors_interface/imu_update` to make them work again.
