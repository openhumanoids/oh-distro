#!/bin/bash

I=lo

sudo tc qdisc del dev $I root
sudo tc qdisc