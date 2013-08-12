#! /bin/bash

echo ""
echo "Palm:"
./parametertest get 29 0

echo ""
echo "Finger 1 Proximal:"
./parametertest get 29 3

echo ""
echo "Finger 1 Distal:"
./parametertest get 29 4

echo ""
echo "Finger 2 Proximal:"
./parametertest get 29 1

echo ""
echo "Finger 2 Distal:"
./parametertest get 29 2

echo ""
echo "Finger 3 Proximal:"
./parametertest get 29 5

echo ""
echo "Finger 3 Distal:"
./parametertest get 29 6

echo ""
echo "F1 Motor:"
./parametertest get 29 10

echo ""
echo "F2 Motor:"
./parametertest get 29 8

echo ""
echo "F3 Motor:"
./parametertest get 29 9

echo ""
echo "F3 Ant. Motor:"
./parametertest get 29 7

echo ""
echo "Tactile:"
./parametertest get 29 11
