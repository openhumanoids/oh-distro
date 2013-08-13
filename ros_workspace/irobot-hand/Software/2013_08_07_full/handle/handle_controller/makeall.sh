#! /bin/bash



cd src
make clean
set -o errexit
make
set +o errexit

cd ../utils
make clean
set -o errexit
make
set +o errexit

cd ..
