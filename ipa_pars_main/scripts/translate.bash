#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ~/git/catkin_ws/src/ipa_pars/ipa_pars_main/tmp/
echo changed directory
echo translating domain and problem files
python ../../ext_planner/adp/translate/translate.py ../../ipa_pars_main/tmp/domain.pddl ../../ipa_pars_main/tmp/problem.pddl &
echo done
