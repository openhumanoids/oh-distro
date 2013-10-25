#! /bin/bash

./graph_F_distal_tactile.sh $1 &
./graph_F_distal_tactile2.sh $1 &
./graph_F_proximal_tactile.sh $1 &
./graph_F_proximal_tactile2.sh $1 &
