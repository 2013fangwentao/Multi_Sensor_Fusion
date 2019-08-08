#!/bin/bash
num=`cat /proc/cpuinfo |grep processor  | wc -l`
num=$[num-1]
echo "make -j$num"
rm -r build
mkdir build && cd build
cmake .. && make -j$num