#!/bin/bash

run_clean()
{
  git reset --hard HEAD
  git clean -fd
}

cd $HOME/drc/software/drake
run_clean

cd $HOME/drc/software/pronto
run_clean

cd $HOME/drc
run_clean

cd $HOME/drc/software
git checkout master
git fetch origin
git reset --hard origin/master
git submodule update --init --recursive

cd $HOME/drc/software
make -j8
