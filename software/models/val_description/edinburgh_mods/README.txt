To add the required frames and contact points to the NASA urdf so that Drake and ddapp can read it:

1. Modify valkyrie_A.urdf to including edinburgh_mods.xacro:
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="valkyrie">
    <xacro:property name="mesh_root" value="package://val_description/meshes"/>
    <!-- <xacro:include filename="$(find val_description)/robots/valkyrie_A/xacro/common/hw_actuator_types.xacro"/> -->
    <xacro:include filename="$(find val_description)/robots/valkyrie_A/xacro/valkyrie_A_sim_base.xacro"/>
    <xacro:include filename="$(find val_description)/robots/valkyrie_A/xacro/edinburgh_mods.xacro"/>
</robot>

2. Compile the urdf (inserting the required content at the bottom)
rosrun xacro xacro.py xacro/valkyrie_A.xacro -o valkyrie_A.urdf

3. move the urdf to this location
cp valkyrie_A.urdf PATH_TO/drc/software/models/val_description/model.urdf

TODO:
Make a fork of NASA's repo and use it instead

- Maurice, june 2015
