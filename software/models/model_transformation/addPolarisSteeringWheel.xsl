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
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <torus radius="0.15" tube_radius="0.01"/>
        </geometry>
      </collision>
    </xsl:copy>
  </xsl:template>
</xsl:stylesheet>
