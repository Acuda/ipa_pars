#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ~/.ros/ipa_pars/output/
echo changed directory
echo preprocessing domain and problem files
./../../../git/catkin_ws/src/ipa_pars/adp_planner/build/adp/preprocess/preprocess < output.sas
echo done

