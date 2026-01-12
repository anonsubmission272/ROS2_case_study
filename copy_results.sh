#! /bin/bash

curIdx=$1

if [ ! -d "results" ]; then
    mkdir results
    echo "Results directory created."
fi

cp _state ./results/state_$curIdx

cp _actuation ./results/actuation_$curIdx