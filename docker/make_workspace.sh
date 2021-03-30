#!/bin/bash

catkin build -w ../
echo "source ../devel/setup.bash" > ~/.bashrc
