#!/bin/bash

pushd ~/drc/software/config/vrc_credentials >& /dev/null&
for i in `find -name "*.pem"`; do chmod 400 $i; done
popd >& /dev/null


