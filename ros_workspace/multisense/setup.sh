#!/usr/bin/env sh
# THIS IS AN AUTO-GENERATED FILE
# IT IS UNLIKELY YOU WANT TO EDIT THIS FILE BY HAND
# IF YOU WANT TO CHANGE THE ROS ENVIRONMENT VARIABLES
# USE THE rosinstall OR rosws TOOL INSTEAD.
# Generator version: 0.6.29
# see: http://www.ros.org/wiki/rosinstall


# This setup.sh file has to parse .rosinstall file, and source similar
# setup.sh files recursively. In the course of recursion, shell
# variables get overwritten. This means that when returning from
# recursion, any variable may be in a different state

# These variables accumulate data through recursion and must only be
# reset and unset at the top level of recursion.

if [ x"$_ROSINSTALL_IN_RECURSION" != x"recurse" ] ; then
  # reset setupfile accumulator
  _SETUPFILES_ROSINSTALL=
  _ROS_PACKAGE_PATH_ROSINSTALL=
  # reset RPP before sourcing other setup files
  export ROS_PACKAGE_PATH=
fi

export ROS_WORKSPACE=/home/mfallon/drc/ros_workspace/multisense
if [ ! "$ROS_MASTER_URI" ] ; then export ROS_MASTER_URI=http://localhost:11311 ; fi
unset ROS_ROOT

unset _SETUP_SH_ERROR

# python script to read .rosinstall even when rosinstall is not installed
# this files parses the .rosinstall and sets environment variables accordingly
# The ROS_PACKAGE_PATH contains all elements in reversed order (for historic reasons)

# We store into _PARSED_CONFIG the result of python code,
# which is the ros_package_path and the list of setup_files to source
# Using python here to benefit of the pyyaml library
export _PARSED_CONFIG=`/usr/bin/env python << EOPYTHON

import sys
import os
import yaml

workspace_path = os.environ.get('ROS_WORKSPACE', os.path.abspath('.'))
filename = os.path.join(workspace_path, '.rosinstall')

if not os.path.isfile(filename):
  print('ERROR')
  sys.exit("There is no file at %s" % filename)

with open(filename, "r") as fhand:
  try:
    v = fhand.read();
  except Exception as e:
    print('ERROR')
    sys.exit("Failed to read file: %s %s " % (filename, str(e)))

try:
  y = yaml.load(v);
except Exception as e:
  print('ERROR')
  sys.exit("Invalid yaml in %s: %s " % (filename, str(e)))

if y is not None:

  # put all non-setupfile entries into ROS_PACKAGE_PATH
  paths = []
  for vdict in y:
    for k, v in vdict.items():
      if v is not None and k != "setup-file":
        path = os.path.join(workspace_path, v['local-name'])
        if not os.path.isfile(path):
          # add absolute path from workspace to relative paths
          paths.append(os.path.normpath(path))
        else:
          print('ERROR')
          sys.exit("ERROR: referenced path is a file, not a folder: %s" % path)
  output = ''
  # add paths in reverse order
  if len(paths) > 0:
    output += ':'.join(reversed(paths))

  # We also want to return the location of any setupfile elements
  output += 'ROSINSTALL_PATH_SETUPFILE_SEPARATOR'
  setupfile_paths = []
  for vdict in y:
    for k, v in vdict.items():
      if v is not None and k == "setup-file":
        path = os.path.join(workspace_path, v['local-name'])
        if not os.path.exists(path):
          print('ERROR')
          sys.exit("WARNING: referenced setupfile does not exist: %s" % path)
        elif os.path.isfile(path):
          setupfile_paths.append(path)
        else:
          print('ERROR')
          sys.exit("ERROR: referenced setupfile is a folder: %s" % path)
  output += ':'.join(setupfile_paths)

  # printing will store the result in the variable
  print(output)
EOPYTHON`

if [ x"$_PARSED_CONFIG" = x"ERROR" ]; then
  echo 'Could not parse .rosinstall file' 1<&2
  _SETUP_SH_ERROR=1
fi

# using sed to split up ros_package_path and setupfile results
_ROS_PACKAGE_PATH_ROSINSTALL_NEW=`echo "$_PARSED_CONFIG" | sed 's,\(.*\)ROSINSTALL_PATH_SETUPFILE_SEPARATOR\(.*\),\1,'`
if [ ! -z "$_ROS_PACKAGE_PATH_ROSINSTALL_NEW" ]; then
  if [ ! -z "$_ROS_PACKAGE_PATH_ROSINSTALL" ]; then
    export _ROS_PACKAGE_PATH_ROSINSTALL=$_ROS_PACKAGE_PATH_ROSINSTALL:$_ROS_PACKAGE_PATH_ROSINSTALL_NEW
  else
    export _ROS_PACKAGE_PATH_ROSINSTALL=$_ROS_PACKAGE_PATH_ROSINSTALL_NEW
  fi
fi

_SETUPFILES_ROSINSTALL_NEW=`echo "$_PARSED_CONFIG" | sed 's,\(.*\)'ROSINSTALL_PATH_SETUPFILE_SEPARATOR'\(.*\),\2,'`
if [ ! -z "$_SETUPFILES_ROSINSTALL_NEW" ]; then
  if [ ! -z "$_SETUPFILES_ROSINSTALL" ]; then
    _SETUPFILES_ROSINSTALL=$_SETUPFILES_ROSINSTALL_NEW:$_SETUPFILES_ROSINSTALL
  else
    _SETUPFILES_ROSINSTALL=$_SETUPFILES_ROSINSTALL_NEW
  fi
fi
unset _PARSED_CONFIG

# colon separates entries
_LOOP_SETUP_FILE=`echo $_SETUPFILES_ROSINSTALL | sed 's,\([^:]*\)[:]\(.*\),\1,'`
# this loop does fake recursion, as the called setup.sh may work on
# the remaining elements in the _SETUPFILES_ROSINSTALL stack
while [ ! -z "$_LOOP_SETUP_FILE" ]
do
  # need to pop from stack before recursing, as chained setup.sh might rely on this
  _SETUPFILES_ROSINSTALL=`echo $_SETUPFILES_ROSINSTALL | sed 's,\([^:]*[:]*\),,'`
  if [ -f "$_LOOP_SETUP_FILE" ]; then
    _ROSINSTALL_IN_RECURSION=recurse
    . $_LOOP_SETUP_FILE
    unset _ROSINSTALL_IN_RECURSION
  else
    echo warn: no such file : "$_LOOP_SETUP_FILE"
  fi
  _LOOP_SETUP_FILE=`echo $_SETUPFILES_ROSINSTALL | sed 's,\([^:]*\)[:]\(.*\),\1,'`
done

unset _LOOP_SETUP_FILE
unset _SETUPFILES_ROSINSTALL

# prepend elements from .rosinstall file to ROS_PACKAGE_PATH
# ignoring duplicates entries from value set by setup files
export ROS_PACKAGE_PATH=`/usr/bin/env python << EOPYTHON
import os
ros_package_path = os.environ.get('ROS_PACKAGE_PATH', '')
original_elements = ros_package_path.split(':')
ros_package_path2 = os.environ.get('_ROS_PACKAGE_PATH_ROSINSTALL', '')
new_elements = ros_package_path2.split(':')
new_elements = [path for path in new_elements if path]

for original_path in original_elements:
  if original_path and original_path not in new_elements:
    new_elements.append(original_path)
print(':'.join(new_elements))
EOPYTHON`

unset _ROS_PACKAGE_PATH_ROSINSTALL

# restore ROS_WORKSPACE in case other setup.sh changed/unset it
export ROS_WORKSPACE=/home/mfallon/drc/ros_workspace/multisense

# if setup.sh did not set ROS_ROOT (pre-fuerte)
if [ -z "${ROS_ROOT}" ]; then
  # using ROS_ROOT now being in ROS_PACKAGE_PATH
  export _ROS_ROOT_ROSINSTALL=`/usr/bin/env python << EOPYTHON
import sys, os;
if 'ROS_PACKAGE_PATH' in os.environ:
  pkg_path = os.environ['ROS_PACKAGE_PATH']
  for path in pkg_path.split(':'):
    if (os.path.basename(path) == 'ros'
        and os.path.isfile(os.path.join(path, 'stack.xml'))):
      print(path)
      break
EOPYTHON`

  if [ ! -z "${_ROS_ROOT_ROSINSTALL}" ]; then
    export ROS_ROOT=$_ROS_ROOT_ROSINSTALL
    export PATH=$ROS_ROOT/bin:$PATH
    export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
  fi
unset _ROS_ROOT_ROSINSTALL
fi

if [ ! -z "$_SETUP_SH_ERROR" ]; then
  # return failure code when sourcing file
  false
fi
