#!/bin/bash
# format_all.sh

PLANNING_ROOT="/home/fanjj1994/ADAS/planning/src/planning_core"
find "$PLANNING_ROOT" -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' -o -name '*.hpp' \) -exec clang-format -i -style=file {} +