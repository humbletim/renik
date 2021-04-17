#!/bin/bash
CXX=${CXX:-clang++}
for x in *.cpp renik/*.cpp; do
  echo $CXX $x
  $CXX -std=c++11 -Ipolyfill -Iglm -c $x -o /dev/null -Wno-return-type -Wno-writable-strings -Wno-macro-redefined -fno-exceptions -Wno-logical-op-parentheses
done
