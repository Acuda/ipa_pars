#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ../tmp/
echo changed directory
echo translating domain and problem files
python ../../build/adp/translate/translate.py ../tmp/domain.pddl ../../ipa_pars_main/tmp/problem.pddl &
echo done
