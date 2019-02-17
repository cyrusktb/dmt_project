#!/bin/bash

# Generate ctags for all files in this project

# Stop on first error
set -e

# Find all files in src and include directories
FILES_TO_TAG=$(find src/ include/ | grep "\.cpp\|\.c") #\.hpp\|\.h")
FILES_TO_TAG="$FILES_TO_TAG main.cpp"
FILES_TO_TAG="${FILES_TO_TAG//$'\n'/ }"
#echo "$FILES_TO_TAG"

# Get headers included by all files and ctags them
g++ -M $FILES_TO_TAG | sed -e 's/[\\ ]/\n/g' | \
        sed -e '/^$/d' -e '/\.o:[ \t]*$/d' | \
        ctags -L - --c++-kinds=+p --fields=+iaS --extra=+q
