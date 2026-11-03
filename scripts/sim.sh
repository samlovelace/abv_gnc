#!/usr/bin/env bash
##########################################################################
# DEV TOOL - Simulation launcher (tmux)
#
# Spins up a tmux session with each node in its own pane for local
# simulation. Not intended for production or hardware use.
#
# Requires: tmux, a built workspace at $WS, and a display for the GUI.
##########################################################################

SESSION=abv-sim
WS=~/dev/cpp/robot_ws/src/autobot

tmux new-session -d -s $SESSION

tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run autobot_controller autobot_controller" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run autobot_navigation autobot_navigation" C-m

tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run autobot_simulator autobot_simulator" C-m

tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run autobot_guidance autobot_guidance" C-m

tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run autobot_commander autobot_commander" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run autobot_gui autobot_gui" C-m

tmux split-window -h -t $SESSION 
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run autobot_bridge autobot_bridge" C-m

tmux select-layout -t $SESSION tiled
tmux attach -t $SESSION
