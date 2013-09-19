#!/bin/bash

ca_test &
matlab -nosplash -r "cd matlab; track_pf();"

exit -1