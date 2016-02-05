#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ~/git/catkin_ws/src/ipa_pars/ipa_pars_main/tmp/
echo changed directory
echo preprocessing domain and problem files
./../../ext_planner/adp/preprocess/preprocess < output.sas
echo done

