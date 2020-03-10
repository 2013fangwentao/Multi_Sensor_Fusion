#!/bin/bash
# get all filename in specified path
 
path=$1
files=$(ls $path)
for filename in $files
do
 echo $filename >> files.txt
done