# ipab-ros-workspace
ROS workspace with common packages used for humanoids research, primarily to provide interoperability between DRC, SCS, and EXOTica as well as the robots we are using.

# Style Guide

## Formatting
For C++, we are using either ROS or a slightly modified BSD/Allman formatting.

To apply automatic formatting, __astyle__ is required: ``sudo apt-get install astyle``. All C++ source and header files can then be formatted by navigating to the directory containing the source and running:

```find -regextype egrep -regex '.*\.[ch](pp)?$' -exec astyle '{}' --style=allman --indent=spaces=2 --pad-oper --unpad-paren --pad-header --convert-tabs \;```

A guide on autoformatting using the Eclipse IDE can be found [here](http://wiki.ros.org/IDEs#Auto_Formatting)

## Linting
We are using [roslint](http://wiki.ros.org/roslint) for static code analysis and to lint our C++ codebase. An example is implemented for the ``drc_translators`` package.

To add roslint to a package, please follow the instructions [here](http://wiki.ros.org/roslint). Afterwards, you can check the code by running ``catkin_make roslint_YOUR_PACKAGE_NAME``, e.g. ``catkin_make roslint_drc_translators``. As a rule of thumb, we are usually removing all errors apart from linelength and when changing is overly bothersome or would inhibit compilation of the codebase.

For a background on why linting is useful, please have a look at the [Wikipedia article](https://en.wikipedia.org/wiki/Lint_(software)), from which the following excerpt is taken:

> In computer programming, lint was the name originally given to a particular program that flagged some suspicious and non-portable constructs (likely to be bugs) in C language source code. The term is now applied generically to tools that flag suspicious usage in software written in any computer language. The term lint-like behavior is sometimes applied to the process of flagging suspicious language usage. Lint-like tools generally perform static analysis of source code.
> Lint as a term can also refer more broadly to syntactic discrepancies in general, especially in interpreted languages like JavaScript and Python. For example, modern lint checkers are often used to find code that doesn't correspond to certain style guidelines. They can also be used as simple debuggers for common errors, or hard to find errors such as heisenbugs.