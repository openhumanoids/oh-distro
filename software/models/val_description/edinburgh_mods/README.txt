***Generating a Drake (designer) compatiable URDF***

To add the required frames and contact points to the NASA urdf so that Drake and ddapp can read it:

1. Use this fork of the official repo:
mitdrc/val_description and branch called: mfallon-mods-needed-for-drake-system

This branch inserts edinburgh_mods.xacro into the xacro.

2. Compile the urdf:
cd val_description/model/robots
rosrun xacro xacro.py valkyrie_A_sim.xacro -o ../urdf/valkyrie_A_sim.urdf

3. Current you need to insert the following inside the link tag for LeftFoot and RightFoot:
This is used for the quasi static balance constraint:

    <collision group="heel">
      <origin rpy="0 0 0" xyz="-0.075 0.0624435 -0.073"/>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
    </collision>
    <visual group="heel">
      <origin rpy="0 0 0" xyz="-0.075 0.0624435 -0.073"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
    </visual>
    <collision group="heel">
      <origin rpy="0 0 0" xyz="-0.075 -0.0624435 -0.073"/>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
    </collision>
    <visual group="heel">
      <origin rpy="0 0 0" xyz="-0.075 -0.0624435 -0.073"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
    </visual>
    <collision group="toe">
      <origin rpy="0 0 0" xyz="0.19 0.0624435 -0.11"/>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
    </collision>
    <visual group="toe">
      <origin rpy="0 0 0" xyz="0.19 0.0624435 -0.11"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
    </visual>
    <collision group="toe">
      <origin rpy="0 0 0" xyz="0.19 -0.0624435 -0.11"/>
      <geometry>
        <sphere radius="0.0"/>
      </geometry>
    </collision>
    <visual group="toe">
      <origin rpy="0 0 0" xyz="0.19 -0.0624435 -0.11"/>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
    </visual>

4. move the urdf to this location
cp valkyrie_A_sim.urdf PATH_TO/drc/software/models/val_description/urdf/valkyrie_A_sim.urdf

5. If needed you can make an sdf file like this:
gzsdf print valkyrie_A.urdf  > valkyrie_A.sdf

6. Generating hands.
Background: the hands located in models/common_components/hand_factory
are independent urdf models and are used when seeding hands in the ddapp ui. 
they are generated from xacro in the same manner as for whole robot:
rosrun xacro xacro.py valkyrie_hand_left.xacro -o ../urdf/valkyrie_hand_left.urdf
rosrun xacro xacro.py valkyrie_hand_right.xacro -o ../urdf/valkyrie_hand_right.urdf
... and then copied into the above locations


***IHMC URDF/SDF***
- There exists 4 files in val_description: valkyrie_A_[hw|sw].[urdf|sdf]
- the urdf isn't read by SCS
- These SDF is: valkyrie_A_sim.sdf
- running the following produces an sdf that is identical to the one loaded by IHMC:
  cd ~/workspace_alpha/ValkyrieHardwareDrivers/bin/models/val_description/urdf
  gzsdf print valkyrie_A_sim.urdf > valkyrie_A_sim.sdf


***3D File Formats***
STL - source from NASA which came to 400MB. NOT USED
DAE - source from NASA (originally made by Marco)
VTP - used by Drake designer
OBJ - required by drake designer to read the vtp files.

Development process:
1 BLEND files combining STL and new meshes created in Blender
2 DAE and OBJ output from Blender 
3 VTP created via script from Pat

1. file extensions to be changed to obj?
2. drake by itself
3. drake designer
4. remove STL
5. changing the normal smoothing file in ddapp


- Maurice, june 2015
