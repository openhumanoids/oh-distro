#!/bin/bash

./run_chroot.py --arch=amd64 --distro=maverick --workspace . --script local_script.bash --ramdisk
./run_chroot.py --arch=i386 --distro=maverick --workspace . --script local_script.bash --ramdisk
./run_chroot.py --arch=amd64 --distro=lucid --workspace . --script local_script.bash --ramdisk
./run_chroot.py --arch=i386 --distro=lucid --workspace . --script local_script.bash --ramdisk


