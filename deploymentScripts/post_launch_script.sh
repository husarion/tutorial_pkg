#!/bin/sh
ssh -i "~/.ssh/outsource_test.pem" -tt ubuntu@ec2-instance /home/ubuntu/ros_workspace/src/tutorial_pkg/scripts/gmapping.sh &
exit 0