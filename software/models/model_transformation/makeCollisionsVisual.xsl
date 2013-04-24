<xsl:stylesheet version="2.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
  <xsl:template name="identity" match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>
  <xsl:template match="collision">
    <xsl:call-template name="identity"/>
    <xsl:element name="visual">
      <xsl:apply-templates select="@*|node()"/>
    </xsl:element>
  </xsl:template>
</xsl:stylesheet>
