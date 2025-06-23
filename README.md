# Racing-CARLA
An open-source simualtor for autonomous racing built upon [CARLA](https://github.com/carla-simulator/carla).

## Setup
R-CARLA heavily depends on [CARLA](https://github.com/carla-simulator/carla). Please first install it by following the instructions in the CARLA repository.


### ROS-Bridge
R-CARLA interfaces with CARLA through a fork of the [ROS-Bridge](https://github.com/maubrunn/ros-bridge). We use a docker container of the bridge and also use its image as the base image for R-CARLA. But before you build the bridge you need to add the definition of your vehicle to `ros-bridge/carla_spawn_objects/config/objects.json`. 

```
git clone https://github.com/maubrunn/ros-bridge.git -b r-carla
cd ros-bridge
git submodule update --init --recursive
cd docker
```
You can now build for ROS1 `./build.sh -r noetic -c YOUR_CARLA_VERSION` or ROS2 `./build.sh -r foxy -c YOUR_CARLA_VERSION`.

### R-CARLA
Now we can finally build R-CARLA by running the following command in the root directory of this repository:
```
docker compose build ros1 OR ros2
```

## Usage
First we need to launch the CARLA server, if you haven't used CARLA before please refer to their documentation for a detailed explanation on how to run CARLA.

We can then start the ros-bridge. It is important that we use CARLA in asynchronous mode because otherwise it can run faster than realtime which can mess up your pipeline:


### ROS1 

```
cd ros-bridge/docker
./run.sh -t noetic roslaunch carla_ros_bridge carla_ros_bridge.launch town:=<YOUR_MAP> register_all_sensors:=False
./run.sh -t noetic roslaunch carla_spawn_objects carla_spawn_objects.launch
```


### ROS2

```
cd ros-bridge/docker
./run.sh -t foxy ros2 launch carla_ros_bridge carla_ros_bridge.launch.py town:=<YOUR_MAP>
./run.sh -t foxy roslaunch carla_spawn_objects carla_spawn_objects.launch.py
```

### R-CARLA
To run R-CARLA we can now call `docker compose up ros1 OR ros2`.
This launches all interfaces of R-CARLA, they can be configured in `cfg/config.yaml`. At the moment the configuration is done for a 1:10 scale racecar such as the F1TENTH cars.

### Config

| Environment      |  |
| ----------- | ----------- |
| env/use_joystick      | Defines if a joystick is used to control the car  |
| env/convert_pc_to_scan| Defines if pointclouds should be converted to laserscan msgs (for 2D lidars)|
|| Traffic Interface |
| num_npc        | Number of spawned npcs |
| map_name        | The used map |
|| Physics Interface |
| vehicle        | Basic parameters of your vehicle such as center of gravity and mass |
| tire_params        | Pacejka Paramters of your vehicle |
||IMU / Odom Interface |
| simulate_imu | Bool to simulate IMU data |
|rate | Rate of the IMU simulation |
| from_simulator | If set to true the IMU simulation is based on the physics simulator, if false it uses the position from CARLA |
|| drive_interface |
| physics | Initial values for the car |
| cam | This are the parameters for the spectator camera that can be set to follow the vehicle |



### Traffic Interface
This spawnes multiple NPC's and moves them along a predefined trajectory. The trajectory is taken from the `global_waypoints.json` file in `interfaces/traffic_interface/maps/MAP_NAME`. The file should have the following structure:

```json
{
    "waypoints": [
        {
            "x": 1.0,
            "y": 0.5,
            "psi": 0.0,
            "vx": 1.0
        },
        {
            "x": 10.0,
            "y": 0.5,
            "psi": 0.0,
            "vx": 1.0,
        },
        ...
    ]
}
```

For each NPC a random spawn point and velocity scaler is chosen.

### Drive Interface
The drive interface moves the ego vehicle in CARLA. At the moment a dynamic single-track model is used. If you want to use custom dynamics this can be done easily. The drive interface calls two services `/rcarla/physics_interface/init` to set an initial position and `/rcarla/physics_interface/step` for each update step. You can add these two services to your custom physics engine and it will be used to update the position of the ego vehicle in CARLA. The [physics interface](interfaces/drive_interface/src/drive_interface/physics_interface.py) should give you an understanding on how to integrate it.

### Sensors Interface
Because we bypass the CARLA dynamics the IMU and wheel odom simualtion do not work anymore. R-CARLA adds this simulation in two ways:

Automatic (`from_simulation=False`)
The position is read from CARLA directly and velocity and acceleration are calculated from the position. This can be very noisy.

Manual (`from_simulation=True`)
The velocities and accelerations are set by the physics simulator. So if your simulator calculates these values you can add a service call to `/rcarla/sensors_interface/odom_update` and `/rcarla/sensors_interface/imu_update`.

## Digital Twin Creation
This module creates a `.obj` file of a pointcloud that can be imported into CARLA as a map. It works with 3D pointclouds but also with pointclouds generated from 2D LiDARs and creates fake walls for these pointclouds. Here is a short description of the process, for a more detailed explanation please see the paper:

* Filtering
* Simplification
* Extension in Z for 2D pointclouds
* Mesh generation

### Usage
In order to create a digital twin of your map you need a pointcloud of it in a `.pcd` file. This can be done by using either a LiDAR or Camera based SLAM system. If you have such a pointcloud you can use the `digital_twin_creator` as follows:

```bash
cd digital_twin_creator
./setup.sh
source venv/bin/activate
python3 digital_twin_creator.py <YOUR_POINTCLOUD_FILE>
```

### Config
The `digital_twin_creator/config.yaml` file has some parameters that you can tune to create a better digital twin.

|Parameter|Description|
|---|---|
|visualize|Bool to toggle visualization|
|outlier_removal | Standard radius outlier removal |
| statistical_outlier_removal | Statistical outlier removal that can be used instead of radius outlier removal |
| mesh/num_samples | Number of samples used in the pointcloud simplifying step |
| mesh/generate_mesh | Bool to toggle mesh generation |
|gpu/force_gpu|Bool to force GPU usage|
|gpu/num_iters|If your pointcloud is too big it will fill up the VRAM, for this the filtering is done in batches|

## Example Usage
An export of CARLA with a F1TENTH car and two maps will follow.
