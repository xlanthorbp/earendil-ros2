#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /real_ws/install/setup.bash

exec "$@"
