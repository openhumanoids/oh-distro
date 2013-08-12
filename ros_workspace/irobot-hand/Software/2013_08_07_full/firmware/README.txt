Firmware for second generation electronics, which support full duplex serial comms

Requires: avr-gcc 8-bit toolchain from atmel
(does not work with avr-gcc Ubuntu packages on Precise Pangolin nor Quantal Quetzal)
get it from: http://www.atmel.com/tools/ATMELAVRTOOLCHAINFORLINUX.aspx

gunzip into /usr/local/avr
  cd /usr/local
  sudo tar -xzf ~/Downloads/avr8-gnu-toolchain-3.4.1.798-linux.any.x86.tar.gz 
  sudo ln -s avr8-gnu-toolchain-linux_x86 avr

add to .bashrc: 
  export PATH=/usr/local/avr/bin:${PATH}

Or if you extract to another location:
  export PATH=$PATH:~/avr8-gnu-toolchain-linux_x86/bin

Note that this path is not: avr8-gnu-toolchain-linux_x86/avr/bin

until 8oct2012, I was using avr8-gnu-toolchain-3.4.0.663
then avr8-gnu-toolchain-3.4.1.798

-----------------

Bootloaders:
  need a recent avrdude (SVN 1104 or later)
  releases available here:
    http://download.savannah.gnu.org/releases/avrdude/
  svn repository here:
    https://savannah.nongnu.org/svn/?group=avrdude
    http://svn.savannah.nongnu.org/viewvc/?root=avrdude

  In each directory there should be an xboot-boot.hex file.
  Program this into the chip with the command:
  ./bootloader/prgx32
  (for the palm, use "make program" or
  sudo avrdude -p atxmega128a1 -P usb -c avrispmkII -e -U boot:w:xboot-boot.hex -U fuse1:w:0x77:m -U fuse2:w:0xBF:m -U fuse4:w:0xF7:m -U fuse5:w:0xF7:m
   )
  After that, "make program" in each directory will upload the application hex file without overwriting the bootloader.
