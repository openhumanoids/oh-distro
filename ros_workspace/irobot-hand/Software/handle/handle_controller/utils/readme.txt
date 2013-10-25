Helpful utilities for the Overo.

Code to be cross-compiled for the Overo.  Currently, commonos-14_1 is 
installed on the Overo, but there is no reason that cannot change.

killhandle.sh: A helpful script to kill the currently running handle_controller.
NOTE: Will also work on the laptop side to kill any process named handle_*.

motortest: interactive program to test low-level motor commands.  specify
motor number, direction, control type, and amount.

parametertest: interactive program to get and set motor parameters.

sensor_poll: specify sensor type and destination, then send a data collection 
command at a rate of 2 Hz and print response.

sensor_spin: print all sensors at a rate of 25 Hz.  (Rate can be adjusted via
command line).

stopall: stop all motors and data collection.

--------
to compile all code:
> source /opt/commonos/commonos-14_1/overoevm/environment-setup
> make

to compile a single executable:
> make <executable name>

--------
to install code on Overo:
> scp <file> root@armH-palm-1:
password is: awareroot

Or just:
> make install

--------
See ../src/readme.txt for more info on networking for the Overo.
