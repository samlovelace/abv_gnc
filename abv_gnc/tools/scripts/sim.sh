#!/usr/bin/env bash

SESSION=robot_stack
WS=~/dev/cpp/robot_ws/src/abv_gnc
WS2=~/dev/python/robot_gui

tmux new-session -d -s $SESSION

# Pane 1
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run abv_controller abv_controller" C-m

# Pane 2
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run abv_navigation abv_navigation" C-m

# Pane 3
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run abv_simulator abv_simulator" C-m

# Pane 4
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run abv_guidance abv_guidance" C-m

tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS && source install/setup.bash && ros2 run abv_commander abv_commander" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION \
  "cd $WS2 && source $WS/install/setup.bash && python3 main.py" C-m

tmux select-layout -t $SESSION tiled
tmux attach -t $SESSION
