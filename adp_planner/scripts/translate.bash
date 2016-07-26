#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ~/.ros/ipa_pars/output/
echo changed directory
echo translating domain and problem files
python ~/git/catkin_ws/src/ipa_pars/adp_planner/build/adp/translate/translate.py ~/.ros/ipa_pars/pddl/domain.pddl ~/.ros/ipa_pars/pddl/problem.pddl &
echo done
