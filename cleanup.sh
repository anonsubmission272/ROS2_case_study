#!/bin/bash

echo "Removing temp state files..."
rm _*
rm missed_*.txt
rm results/actuation_*
rm results/state_*
echo "Finished"
