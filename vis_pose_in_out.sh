#!/bin/bash
session="IARC-testing"
tmux new-session -d -s $session
window=0
#tmux split-window -t $session:$window -h -p 80
tmux send-keys -t $session:$window 'python3 scripts/vis_localpose.py' C-m
tmux split-window -t $session:$window -h -p 50 
tmux send-keys -t $session:$window 'python3 scripts/vis_setpoint.py' C-m
tmux attach-session -t $session
