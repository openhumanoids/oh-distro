#!/bin/bash

pushd ~/drc/cloudsim >& /dev/null&
for i in `find -name "*.pem"`; do chmod 400 $i; done
popd >& /dev/null


