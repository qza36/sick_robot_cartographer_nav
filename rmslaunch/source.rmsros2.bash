#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/rmsros2/sickscanxd/install/setup.bash
source /root/rmsros2/cartographer/2.0.9003/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/rmsros2/sickscanxd/CycloneDDS.xml"




