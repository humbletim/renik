#!/bin/bash
CXX=${CXX:-c++}
for x in *.cpp renik/*.cpp; do
  echo $CXX $x
  $CXX -std=c++11 -Ipolyfill -Iglm -c $x -o /dev/null
done
