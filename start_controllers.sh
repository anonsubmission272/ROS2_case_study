#!/bin/bash

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting all three controllers..."

./build/tmp0 &
PID0=$!
./build/tmp1 &
PID1=$!
./build/tmp2 &
PID2=$!

echo "Controllers started. PIDs: $PID0 $PID1 $PID2"

cleanup() {
  echo "Stopping controllers..."
  kill $PID0 $PID1 $PID2
  wait $PID0 $PID1 $PID2
  echo "Controllers stopped"
}

trap cleanup EXIT INT TERM

echo "Press Ctrl+C to stop all controllers"
wait

