To add the required frames and contact points to the NASA urdf so that Drake and ddapp can read it:

1. Use this fork of the official repo:
mitdrc/val_description and branch called: mfallon-mods-needed-for-drake-system

This branch inserts edinburgh_mods.xacro into the xacro.

2. Compile the urdf as suggested:
rosrun xacro xacro.py xacro/valkyrie_A.xacro -o valkyrie_A.urdf

3. Current you need to insert the following inside the link tag or LeftFoot and RightFoot:
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
cp valkyrie_A.urdf PATH_TO/drc/software/models/val_description/model.urdf


- Maurice, june 2015
