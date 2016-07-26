#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ~/.ros/ipa_pars/output/
echo changed directory
echo calling planner and solving planning problem
python ~/git/catkin_ws/src/ipa_pars/adp_planner/build/adp/fast-downward.py output --heuristic 'hff=adp(cost_type=1)' --search 'lazy_greedy(hff, preferred=hff)' &
echo

