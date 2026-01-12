#! /bin/bash

export WORKSPACE_ROOT="."
export CONTROLLER_DIR="$WORKSPACE_ROOT/src/example_7/controller"

echo "Building controller 0"

gcc -c -o $CONTROLLER_DIR/c0/controller.o $CONTROLLER_DIR/c0/controller.c
g++ -c -o $CONTROLLER_DIR/c0/controller_main.o $CONTROLLER_DIR/c0/controller_main.cpp
g++ -o $CONTROLLER_DIR/tmp0 $CONTROLLER_DIR/c0/controller.o $CONTROLLER_DIR/c0/controller_main.o

cp $CONTROLLER_DIR/tmp0 $WORKSPACE_ROOT/build/tmp0

echo "Finished building controller 0"

