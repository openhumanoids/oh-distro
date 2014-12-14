#!/bin/sh

lcm-gen -j ../lcmtypes/*.lcm

javac -cp ../lcm-java/lcm.jar drc/*.java

jar cf my_types.jar drc/*.class
