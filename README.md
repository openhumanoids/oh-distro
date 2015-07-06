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
