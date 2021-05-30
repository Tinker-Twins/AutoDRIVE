#!/bin/bash
# Script to clean the project tree from all compiled files.
# Rebuild the project using "build.sh".

# Go into the directory where this bash script is contained.
cd `dirname $0`

# Remove the dedicated output directory.
rm -rf build

echo Project Cleaned
