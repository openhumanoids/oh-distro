# ipab-distro
This repository checks out working tandems of the ``drc`` and ``ipab-ros-workspace`` codebases. 

To get started:

```
cd ~
git clone git@github.com:ipab-slmc/ipab-distro.git
cd ipab-distro
git submodule update --init --recursive
```

To contribute:

 1. Create a new branch with your feature in the corresponding submodule
 1. Make a pull request against the corresponding submodule
 1. Once the PR of the corresponding submodule is accepted, create a branch of ipab-distro and forward the submodule tip, and make a pull request against ipab-distro/master with a working combination of submodules


## Optional modules
This distro does not include the Kuka LWR and Schunk SDH repositories that are only needed on the respective computers running these components. These are at the time of writing ``wendy`` for the Kuka LWR driver and ``mindy`` for the Schunk SDH driver. The source code for the packages can be found here: [Kuka LWR](https://bitbucket.org/IPAB-SLMC/kuka-lwr/), [Schunk SDH](https://bitbucket.org/IPAB-SLMC/schunk-sdh).
