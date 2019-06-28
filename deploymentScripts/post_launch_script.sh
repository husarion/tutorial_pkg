#!/bin/sh
ssh -i "~/.ssh/rosbot_key.pem" -tt ubuntu@ec2-instance /home/ubuntu/ros_workspace/src/tutorial_pkg/scripts/gmapping.sh &
exit 0