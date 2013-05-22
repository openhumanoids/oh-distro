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
    <xsl:copy-of select="current()"/>
    <xsl:value-of select="$newline"/>
    <xsl:element name="collision">
      <xsl:apply-templates select="@*|pose|origin|geometry"/>
    </xsl:element>
  </xsl:template>
</xsl:stylesheet>
