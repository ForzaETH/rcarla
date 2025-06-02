#!/bin/bash
set -e

# setup ros environment

CONFIG_PATH="/opt/ws/src/cfg/config.yaml"
ROS2_TMP_CONFIG="/opt/ws/src/cfg/ros2_config.yaml"
source "/opt/ws/install/setup.bash"
source "/opt/carla/setup.bash"

if [  "$ROS_VERSION" == "1" ]; then
    rosparam load "$CONFIG_PATH"
    export USE_JOYSTICK=$(rosparam get /env/use_joystick)
else 
    echo "Converting config to ROS 2 format..."    
    python /parse_ros2_params.py $CONFIG_PATH $ROS2_TMP_CONFIG
    echo "Generated ROS 2 param file:"
    cat "$ROS2_TMP_CONFIG"
    export USE_JOYSTICK=$(python3 -c "
        import yaml
        with open('$CONFIG_PATH') as f:
            data = yaml.safe_load(f)
        print(data.get('env', {}).get('use_joystick', False))
        ")
fi


exec "$@"
