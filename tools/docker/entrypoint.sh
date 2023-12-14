#!/bin/bash
set -e

source "/opt/ros/noetic/setup.bash" --
source "/ws/install/setup.bash" --
exec "$@"
