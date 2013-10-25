#! /bin/bash

echo ""
echo "Controller:"
./handle_controller -v

echo ""
echo "Palm (device 0):"
./parametertest get 29 0

echo ""
echo "Finger 1 Proximal (device 3):"
./parametertest get 29 3

echo ""
echo "Finger 1 Distal (device 4):"
./parametertest get 29 4

echo ""
echo "Finger 2 Proximal (device 1):"
./parametertest get 29 1

echo ""
echo "Finger 2 Distal (device 2):"
./parametertest get 29 2

echo ""
echo "Finger 3 Proximal (device 5):"
./parametertest get 29 5

echo ""
echo "Finger 3 Distal (device 6):"
./parametertest get 29 6

echo ""
echo "F1 Motor (device 10):"
./parametertest get 29 10

echo ""
echo "F2 Motor (device 8):"
./parametertest get 29 8

echo ""
echo "F3 Motor (device 9):"
./parametertest get 29 9

echo ""
echo "F3 Ant. Motor (device 7):"
./parametertest get 29 7

echo ""
echo "Tactile (device 11):"
./parametertest get 29 11
