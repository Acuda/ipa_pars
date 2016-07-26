#!/bin/bash
echo sourcing ...
source ~/.bashrc
echo starting ...
cd ../tmp/
echo changed directory
echo preprocessing domain and problem files
./../../build/adp/preprocess/preprocess < output.sas &
echo done

