#!/bin/bash
#
# Converts V1 XACRO files to URDF files for the model and xacro files specified
# in arguments of make_xacros. The first argument is the package name, the rest
# of the arguments are model types to build (these are the strings appended to
# the model name in the models folder for V1 models.
#
# Author: R. W. Sinnet (ryan@rwsinnet.com)

make_xacros()
{
    local MODEL_PREFIX=$1
    shift
    local XACROS=($@)
    local XACRO_DIR=$(rospack find ${MODEL_PREFIX})/models/${MODEL_PREFIX}/xacro
    local URDF_DIR=$(rospack find $MODEL_PREFIX)/models/${MODEL_PREFIX}/urdf

    local OLD_IFS=$IFS
    IFS=,
    echo "Building the following URDFs from XACROs for model ${MODEL_PREFIX}:\
 (${XACROS[*]})"
    echo
    mkdir -p $URDF_DIR
    IFS=$OLD_IFS
    for MODEL_SUFFIX in "${XACROS[@]}"
    do
    {
    	local XACRO_NAME="${XACRO_DIR}/${MODEL_PREFIX}_${MODEL_SUFFIX}.xacro"
    	local URDF_NAME="${URDF_DIR}/${MODEL_PREFIX}_${MODEL_SUFFIX}.urdf"
    	local COMMAND="rosrun xacro xacro.py $XACRO_NAME -o $URDF_NAME"
    	echo $COMMAND
    	$COMMAND
    } &
    done
    wait
    echo "Done generating URDFs!"
}
