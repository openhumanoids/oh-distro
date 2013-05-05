<xsl:stylesheet version="2.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

  <xsl:output method="xml" indent="yes"/>

  <xsl:variable name="newline"><xsl:text>
  </xsl:text></xsl:variable>

  <xsl:template name="identity" match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>
  <xsl:template match="link/visual">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
      <xsl:value-of select="$newline"/>
      <material>
      <xsl:value-of select="$newline"/>
        <script>
      <xsl:value-of select="$newline"/>
          <uri>file://media/materials/scripts/gazebo.material</uri>
      <xsl:value-of select="$newline"/>
          <name>Gazebo/Wood</name>
      <xsl:value-of select="$newline"/>
        </script>
      <xsl:value-of select="$newline"/>
      </material>
      <xsl:value-of select="$newline"/>
    </xsl:copy>
  </xsl:template>
</xsl:stylesheet>
