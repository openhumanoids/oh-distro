Code to be cross-compiled for the Overo.  Currently, commonos-14_1 is 
installed on the Overo, but there is no reason that cannot change.

--------
to compile code:
> source /opt/commonos/commonos-14_1/overoevm/environment-setup
> make

--------
to install code on Overo:
> scp handle_controller root@armH-palm-1:
password is: awareroot

Or just:
> make install

--------
The hand has a static IP of: 192.168.33.22
You should put yourself on the hand's subnet:
> sudo ifconfig eth0:armh 192.168.33.10 netmask 255.255.255.0

You may want to put this in your /etc/network/interfaces file:
auto eth0:armh
iface eth0:armh inet static
      address 192.168.33.10
      netmask 255.255.255.0

You may want to put this in your /etc/hosts file:
192.168.33.22 armH-palm-1

--------
To run the code on the Overo:
The Overo is set up to run: /home/root/handle_controller on startup.
If you send new code over, you will have to stop the currently running 
executable, and restart it.  (Actually, sending the code to the Overo
will fail if you don't stop the executable before copying the code).

