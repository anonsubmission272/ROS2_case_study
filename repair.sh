#!/bin/bash

bad_num=$1

# max_idx for test case generation
max_idx=$2

here_dir=$(pwd)

cp src/example_7/controller/c$bad_num/controller.c docker/controller.c

python3 make_testcases.py $1 $max_idx

# cp -r docker/ repair_$bad_num/
# cp repair.yml repair_$bad_num/

make

# FIXME: Update your path to Darjeeling
cd [REDACTED FOR ANONYMITY]

# IDK if this will work... 
pipenv run $here_dir/repair_helper.sh $bad_num $here_dir
