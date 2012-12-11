#!/bin/bash
 # echo of all files in a directory

for file in *.stl
do
  name=${file%%[.]*}
  meshlabserver -i $file -o $name'_chull.stl' -om vn -s chull.mlx
  #echo $name'.obj' 
done
