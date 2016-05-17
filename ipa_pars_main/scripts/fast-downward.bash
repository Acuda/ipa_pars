#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ~/git/catkin_ws/src/ipa_pars/ipa_pars_main/tmp/
echo changed directory
echo calling planner and solving planning problem
python ../../ext_planner/adp/fast-downward.py output --heuristic 'hff=adp(cost_type=1)' --search 'lazy_greedy(hff, preferred=hff)' &
echo

