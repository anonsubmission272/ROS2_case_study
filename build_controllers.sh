#! /bin/bash

export WORKSPACE_ROOT="."
export CONTROLLER_DIR="$WORKSPACE_ROOT/src/example_7/controller"

echo "Building all three controllers"

gcc -c -o $CONTROLLER_DIR/c0/controller.o $CONTROLLER_DIR/c0/controller.c
g++ -c -o $CONTROLLER_DIR/c0/controller_main.o $CONTROLLER_DIR/c0/controller_main.cpp
g++ -o $CONTROLLER_DIR/tmp0 $CONTROLLER_DIR/c0/controller.o $CONTROLLER_DIR/c0/controller_main.o
gcc -c -o $CONTROLLER_DIR/c1/controller.o $CONTROLLER_DIR/c1/controller.c
g++ -c -o $CONTROLLER_DIR/c1/controller_main.o $CONTROLLER_DIR/c1/controller_main.cpp
g++ -o $CONTROLLER_DIR/tmp1 $CONTROLLER_DIR/c1/controller.o $CONTROLLER_DIR/c1/controller_main.o
gcc -c -o $CONTROLLER_DIR/c2/controller.o $CONTROLLER_DIR/c2/controller.c
g++ -c -o $CONTROLLER_DIR/c2/controller_main.o $CONTROLLER_DIR/c2/controller_main.cpp
g++ -o $CONTROLLER_DIR/tmp2 $CONTROLLER_DIR/c2/controller.o $CONTROLLER_DIR/c2/controller_main.o

cp $CONTROLLER_DIR/tmp0 $WORKSPACE_ROOT/build/tmp0
cp $CONTROLLER_DIR/tmp1 $WORKSPACE_ROOT/build/tmp1
cp $CONTROLLER_DIR/tmp2 $WORKSPACE_ROOT/build/tmp2

echo "Finished building all three controllers. Start them with './start_controllers.sh'"

