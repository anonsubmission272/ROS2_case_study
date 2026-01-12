#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Must provide vuln to copy."
else
    cp ./vulns/$1/controller.c ./src/example_7/controller/c0/controller.c
    cp ./vulns/$1/send_trajectory.cpp ./src/example_7/reference_generator/send_trajectory.cpp
fi
