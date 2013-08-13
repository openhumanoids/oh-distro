#!/bin/bash
set -o verbose
bin/sandia_hand_cli fp 0 low
sleep 1
bin/sandia_hand_cli fp 0 on
sleep 4
bin/sandia_hand_cli pp 0 on
sleep 4
bin/sandia_hand_cli fping 0
bin/sandia_hand_cli fp 0 off
