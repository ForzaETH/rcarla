#!/bin/bash
set -e

# setup ros environment

# if [  "$ROS_VERSION" == "2" ]; then
#     source "/opt/ws/install/setup.bash"
# else
#     source "/opt/ws/devel/setup.bash"
# fi
source "/opt/ws/install/setup.bash"

source "/opt/carla/setup.bash"

exec "$@"
