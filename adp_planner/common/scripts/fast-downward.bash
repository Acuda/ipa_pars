#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
echo change directory
cd ../tmp/
echo calling planner and solving planning problem
python ../../build/adp/fast-downward.py output --heuristic 'hff=adp(cost_type=1)' --search 'lazy_greedy(hff, preferred=hff)' &
echo called planner ready

