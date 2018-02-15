#!/bin/bash

rostopic pub /pose_name std_msgs/String "data: 'press4'" -1
sleep 6
rostopic pub /pose_name std_msgs/String "data: 'start_coffee'" -1
sleep 8
rostopic pub /pose_name std_msgs/String "data: 'stop_coffee'" -1
sleep 2
rostopic pub /pose_name std_msgs/String "data: 'press4'" -1



