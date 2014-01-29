#!/bin/bash

sudo route del -net 224.0.0.0 netmask 240.0.0.0 dev lo
