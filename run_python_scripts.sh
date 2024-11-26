#!/bin/bash

# Create a new tmux session named "python_scripts" and run the first command
tmux new-session -d -s python_scripts 'python3 segment_human.py'

# Split the window and run the second command
tmux split-window -v 'python3 cmd_robot.py'
tmux select-layout tiled

# Split the window for the third command
tmux split-window -v 'python3 measure_distance.py'

# Split the window for the fourth command
tmux split-window -v 'python3 measure_command.py'

# Attach to the tmux session to monitor the output
tmux attach-session -t python_scripts

