#!/bin/bash
 # echo of all files in a directory

for file in *.dae
do
  name=${file%%[.]*}
  meshlabserver -i $file -o $name'_chull.obj' -om vn -s chull.mlx
  #echo $name'.obj' 
done
