#!/bin/sh
echo "Launch remote gmapping"
ssh -i "~/.ssh/rosbot_key.pem" ubuntu@ec2-instance /home/ubuntu/ros_workspace/src/tutorial_pkg/scripts/gmapping.sh
