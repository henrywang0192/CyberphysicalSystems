#!/bin/bash
tmux new-session -d -s roscore && \
tmux new-session -d -s roslaunch ./src/race/src/basiclanuch.launch && \
tmux new-session -d -s rosrun race dist_finder && \
tmux new-session -d -s rosrun race control && \
tmux new-session -d -s rosrun zed_wrapper zed_wrapper_node && \
tmux new-session -d -s rosrun color_tracking color_tracking_node
