#!/bin/bash
# format_all.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PLANNING_ROOT="$SCRIPT_DIR/../src/planning_core"
find "$PLANNING_ROOT" -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' -o -name '*.hpp' \) -exec clang-format -i -style=file {} +