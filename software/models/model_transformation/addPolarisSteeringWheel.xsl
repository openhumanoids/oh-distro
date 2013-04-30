<xsl:stylesheet version="2.0" 
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:xs="http://www.w3.org/2001/XMLSchema" >

  <xsl:output method="xml" indent="yes"/>

  <xsl:template match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

  <xsl:template match="link[@name='steering_wheel']">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
      <inertial>
        <mass value="1.0" />
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="1.0" />
      </inertial>
      <collision name="steering_wheel_post_left">
        <origin rpy=" 1.570796 0 0" xyz="0.0 0.08 0.0"/> 
        <geometry>
          <cylinder radius="0.02" length="0.15" />
        </geometry>
      </collision>
      <collision name="steering_wheel_post_right">
        <origin rpy=" 1.570796 0 0" xyz="0.0 -0.08 0.0"/> 
        <geometry>
          <cylinder radius="0.02" length="0.15" />
        </geometry>
      </collision>
      <collision name="steering_wheel_post_middle">
        <origin rpy="0 1.570796 0" xyz="-0.08 0.0 0.0"/> 
        <geometry>
          <cylinder radius="0.02" length="0.15" />
        </geometry>
      </collision>
      <collision name="collision00">
        <origin rpy=" 0 1.570796 1.570796" xyz="0.150000 0.000000 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision01">
        <origin rpy=" 0 1.570796 2.199115" xyz="0.121353 0.088168 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision02">
        <origin rpy=" 0 1.570796 2.827433" xyz="0.046353 0.142658 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision03">
        <origin rpy=" 0 1.570796 3.455752" xyz="-0.046353 0.142658 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision04">
        <origin rpy=" 0 1.570796 4.084070" xyz="-0.121353 0.088168 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision05">
        <origin rpy=" 0 1.570796 4.712389" xyz="-0.150000 0.000000 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision06">
        <origin rpy=" 0 1.570796 5.340708" xyz="-0.121353 -0.088168 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision07">
        <origin rpy=" 0 1.570796 5.969026" xyz="-0.046353 -0.142658 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision08">
        <origin rpy=" 0 1.570796 6.597345" xyz="0.046353 -0.142658 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
      <collision name="collision09">
        <origin rpy=" 0 1.570796 7.225663" xyz="0.121353 -0.088168 0"/>
        <geometry><cylinder length="0.105065778087" radius="0.01" /></geometry>
      </collision>
    </xsl:copy>
  </xsl:template>
</xsl:stylesheet>
