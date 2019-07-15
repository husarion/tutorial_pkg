#!/bin/sh
echo "Start post launch"
nohup ssh -i "~/.ssh/rosbot_key.pem"  ubuntu@ec2-instance /home/ubuntu/ros_workspace/src/tutorial_pkg/scripts/gmapping.sh >/dev/null 2>&1 &
sleep 1
echo "SSH finished"
exit 0