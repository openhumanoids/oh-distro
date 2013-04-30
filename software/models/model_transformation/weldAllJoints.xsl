<xsl:stylesheet version="2.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

  <xsl:output method="xml" indent="yes"/>

  <xsl:variable name="newline"><xsl:text>
  </xsl:text></xsl:variable>

  <xsl:template name="identity" match="@*|node()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()"/>
    </xsl:copy>
  </xsl:template>

  <xsl:template match="joint">
    <xsl:copy>
      <xsl:attribute name="type">
        <xsl:value-of select="'fixed'"/>
      </xsl:attribute>
      <xsl:apply-templates select="@* except @type|node()"/>
    </xsl:copy>
  </xsl:template>
</xsl:stylesheet>
