#! /bin/bash

for d in 7 8 9 10
do
    ./parametertest2 s 11 90 $d
    ./parametertest2 s 12 90 $d    
    ./parametertest2 s 17 90 $d
    ./parametertest2 s 14 110 $d
    ./parametertest2 s 20 150 $d
    ./parametertest2 s 31 0 $d
done

./parametertest2 s 11 90 m
./parametertest2 s 12 90 m
./parametertest2 s 17 90 m
./parametertest2 s 14 110 m
./parametertest2 s 20 150 m
./parametertest2 s 31 0 m

for d in 1 2
do
    ./parametertest2 s 26 1000 p
    ./parametertest2 s 27 10 p
    ./parametertest2 s 31 0 p
    ./parametertest2 s 31 0 t
done
