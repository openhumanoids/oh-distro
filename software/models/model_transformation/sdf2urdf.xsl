<xsl:stylesheet version="2.0" 
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:xs="http://www.w3.org/2001/XMLSchema"
  xmlns:mat="http://www.mit.edu"
  xmlns:trans="http://www.mit.edu"
  xmlns:func="http://exslt.org/functions" 
  xmlns:math="http://exslt.org/math" 
  extension-element-prefixes="math func">

  <xsl:output method="xml" indent="yes"/>


  <func:script implements-prefix="math" 
    language="exslt:javascript" 
    src="exslt_math.js" />

  <xsl:variable name="newline"><xsl:text>
  </xsl:text></xsl:variable>

  <!--Pass comments through from .sdf-->
  <xsl:template match="comment()">
    <xsl:comment><xsl:value-of select="."/></xsl:comment>
  </xsl:template>

  <!--Root and sdf-->
  <xsl:template match="/"> <xsl:apply-templates/> </xsl:template>
  <xsl:template match="sdf"> <xsl:apply-templates/> </xsl:template>

  <!--Tags that aren't supported-->
  <xsl:template match="*[exists(geometry/mesh)]"/>
  <xsl:template match="material"/>
  <xsl:template match="surface"/>
  <xsl:template match="physics/ode"/>

  <!--======================================================================-->
  <!--Elements with special transformations-->
  <!--======================================================================-->
  <!--model ==> object-->
  <xsl:template match="model">
    <robot name="{@name}">
      <xsl:apply-templates/>
    </robot>
  </xsl:template>

  <!--link ==> link + joint-->
  <xsl:template match="link">

    <xsl:variable name="link-name" select="@name"/>

    <!--Find joint-->
    <xsl:variable name="current-joint"
      select="../joint[child/text() = $link-name]"/>

    <!--Find parent link-->
    <xsl:variable name="parent" select="/sdf/model/link[@name = $current-joint/parent/text()]"/>

    <!--Get poses-->
    <xsl:variable name="child-pose"   select="pose/text()"/>
    <xsl:variable name="parent-pose"  select="$parent/pose/text()"/>
    <xsl:variable name="joint-pose"   select="$current-joint/pose/text()"/>
  
    <!--Convert to transforms-->
    <xsl:variable name="o_X_c" as="xs:double*">
      <xsl:choose>
        <xsl:when test="exists($child-pose)">
          <xsl:sequence select="trans:pose2transform(mat:V($child-pose))"/>
        </xsl:when>
        <xsl:otherwise>
          <xsl:sequence select="trans:identity()"/>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:variable>

    <xsl:variable name="o_X_p" as="xs:double*">
      <xsl:choose>
        <xsl:when test="exists($parent-pose)">
          <xsl:sequence select="trans:pose2transform(mat:V($parent-pose))"/>
        </xsl:when>
        <xsl:otherwise>
          <xsl:sequence select="trans:identity()"/>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:variable>

    <xsl:variable name="c_X_j" as="xs:double*">
      <xsl:choose>
        <xsl:when test="exists($joint-pose)">
          <xsl:sequence select="trans:pose2transform(mat:V($joint-pose))"/>
        </xsl:when>
        <xsl:otherwise>
          <xsl:sequence select="trans:identity()"/>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:variable>

    <!--Compute transform from world frame to parent frame-->
    <xsl:variable name="p_X_o" select="trans:inv($o_X_p)" as="xs:double*"/>

    <!--Compute transform from child frame to joint frame-->
    <xsl:variable name="j_X_c" select="trans:inv($c_X_j)" as="xs:double*"/>

    <!--Compute transform from joint frame to parent frame-->
    <xsl:variable name="p_X_j" as="xs:double*">
      <xsl:sequence select="trans:compound($p_X_o,trans:compound($o_X_c,$c_X_j))"/>
    </xsl:variable>

    <!--Compute transform from world frame to joint frame-->
    <xsl:variable name="j_X_o" as="xs:double*">
      <xsl:sequence select="trans:compound($j_X_c,trans:inv($o_X_c))"/>
    </xsl:variable>

    <!--Compute transform from child frame to parent frame-->
    <xsl:variable name="p_X_c" as="xs:double*">
      <xsl:sequence select="trans:compound($p_X_o,$o_X_c)"/>
    </xsl:variable>

    <!--Create link-->
    <xsl:copy>
      <xsl:for-each select="@*">
        <xsl:attribute name="{name()}">
          <xsl:value-of select="."/>
        </xsl:attribute>
      </xsl:for-each>
      <xsl:apply-templates select="visual | collision | inertial">
        <xsl:with-param name="j_T_c" select="$j_X_c"/>
      </xsl:apply-templates>
    </xsl:copy>

    <!--Create joint-->
    <xsl:if test="exists($current-joint)">
      <xsl:for-each select="../joint[child/text() = $link-name]">
        <xsl:call-template name="transform-joint">
          <!--<xsl:with-param name="p_X_j" select="$p_X_j"/>-->
          <xsl:with-param name="p_X_j" select="$p_X_c"/>
          <xsl:with-param name="j_X_o" select="$j_X_o"/>
          <!--<xsl:with-param name="j_X_o" select="trans:inv($o_X_c)"/>-->
        </xsl:call-template>
      </xsl:for-each>
    </xsl:if>
    <!--<xsl:value-of select="$newline" />-->
    <!--<xsl:value-of select="$o_X_p"/>-->
    <!--<xsl:value-of select="$newline" />-->
    <!--<xsl:value-of select="$o_X_c"/>-->
    <!--<xsl:value-of select="$newline" />-->
    <!--<xsl:value-of select="$c_X_j"/>-->
    <!--<xsl:value-of select="$newline" />-->
    <!--<xsl:value-of select="trans:getR($j_X_o)"/>-->
  </xsl:template>

  <!--We only want the joints in the tree-->
  <xsl:template match="joint"/>

  <!--transform ==> origin-->
  <xsl:template name="origin">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:element name="origin">
      <xsl:attribute name="xyz">
        <xsl:value-of select="trans:transform2xyz($T)"/>
      </xsl:attribute>
      <xsl:attribute name="rpy">
        <xsl:value-of select="trans:transform2rpy($T)"/>
      </xsl:attribute>
    </xsl:element>
  </xsl:template>


  <!--parent ==> parent; child ==> child-->
  <xsl:template match="parent | child">
    <xsl:copy>
      <xsl:attribute name="link">
        <xsl:value-of select="text()"/>
      </xsl:attribute>
      <xsl:attribute name="type">
        <xsl:value-of select="'link'"/>
      </xsl:attribute>
    </xsl:copy>
  </xsl:template>


  <!--joint ==> joint-->
  <xsl:template name="transform-joint">
    <xsl:param name="p_X_j" as="xs:double*"/>
    <xsl:param name="j_X_o" as="xs:double*"/>
    <xsl:element name="joint">
      <xsl:attribute name="name">
        <xsl:value-of select="@name"/>
      </xsl:attribute>
      <xsl:choose>
        <xsl:when test="@type = 'revolute' and not(exists(axis/limit))">
          <xsl:attribute name="type">
            <xsl:value-of select="'continuous'"/>
          </xsl:attribute>
        </xsl:when>
        <xsl:otherwise>
          <xsl:attribute name="type">
            <xsl:value-of select="@type"/>
          </xsl:attribute>
        </xsl:otherwise>
      </xsl:choose>
      <xsl:call-template name="origin">
        <xsl:with-param name="T" select="$p_X_j"/>
      </xsl:call-template>
      <xsl:for-each select="axis">
        <xsl:call-template name="axis">
          <xsl:with-param name="T" select="$j_X_o"/>
        </xsl:call-template>
      </xsl:for-each>
      <xsl:apply-templates select="*[not(self::pose|self::axis)]"/>
    </xsl:element>
  </xsl:template>

  <!--axis ==> axis + [limit]-->
  <xsl:template name="axis">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:variable name="Vxyz" select="mat:V(xyz/text())"/>
    <xsl:copy>
      <xsl:attribute name="xyz">
        <xsl:value-of select="mat:M33timesV3(trans:getR($T),$Vxyz)"/>
      </xsl:attribute>
    </xsl:copy>
    <xsl:apply-templates select="limit"/>
  </xsl:template>
  
  <!--======================================================================-->
  <!--limit ==> limit-->
  <!--======================================================================-->
  <xsl:template match="axis/limit">
    <xsl:copy>
      <xsl:for-each select="@*">
        <xsl:attribute name="{name()}">
          <xsl:value-of select="."/>
        </xsl:attribute>
      </xsl:for-each>
      <xsl:attribute name="effort">
        <xsl:value-of select="'1'"/>
      </xsl:attribute>
      <xsl:attribute name="velocity">
        <xsl:value-of select="'1'"/>
      </xsl:attribute>
      <xsl:for-each select="*">
        <xsl:attribute name="{name()}">
          <xsl:value-of select="text()"/>
        </xsl:attribute>
      </xsl:for-each>
    </xsl:copy>
  </xsl:template>

  <!--======================================================================-->
  <!--Groups of elements with the same transformations-->
  <!--======================================================================-->
  <!--Copy all attributes and apply templates to child elements-->
  <xsl:template match="collision | visual | inertial | geometry">
    <xsl:param name="j_T_c" as="xs:double*"/>
    <xsl:copy>
      <xsl:for-each select="@*">
        <xsl:attribute name="{name()}">
          <xsl:value-of select="."/>
        </xsl:attribute>
      </xsl:for-each>

      <xsl:if test="exists($j_T_c)">
        <xsl:variable name="element-pose"   select="pose/text()"/>
        <!--Convert to transform-->
        <xsl:variable name="c_T_e" as="xs:double*">
          <xsl:choose>
            <xsl:when test="exists($element-pose)">
              <xsl:sequence select="trans:pose2transform(mat:V($element-pose))"/>
            </xsl:when>
            <xsl:otherwise>
              <xsl:sequence select="trans:identity()"/>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:variable>

        <xsl:variable name="j_T_e" select="trans:compound($j_T_c,$c_T_e)" as="xs:double*"/>
        <xsl:call-template name="origin">
          <xsl:with-param name="T" select="$j_T_e"/>
        </xsl:call-template>
      </xsl:if>
      <xsl:apply-templates select="*[not(self::pose)]"/>
    </xsl:copy>
  </xsl:template>

  <!--Convert all child elements to attributes for these tags-->
  <xsl:template match="box | cylinder | inertia">
    <xsl:copy>
      <xsl:for-each select="*">
        <xsl:attribute name="{name()}">
          <xsl:value-of select="text()"/>
        </xsl:attribute>
      </xsl:for-each>
    </xsl:copy>
  </xsl:template>

  <!--Convert the text of these tags to the attribute "value"-->
  <xsl:template match="mass">
    <xsl:copy>
      <xsl:attribute name="value">
        <xsl:value-of select="text()"/>
      </xsl:attribute>
    </xsl:copy>
  </xsl:template>

  <xsl:function name="mat:V" as="xs:double*">
    <xsl:param name="str" as="xs:string"/>
    <xsl:variable name="str-seq" select="tokenize($str,'\s+')"/>
    <xsl:for-each select="$str-seq">
      <xsl:sequence select="xs:double(current())"/>
    </xsl:for-each>
  </xsl:function>

  <xsl:function name="mat:M33timesV3" as="xs:double*">
    <xsl:param name="mat" as="xs:double*"/>
    <xsl:param name="vec" as="xs:double*"/>
    <xsl:sequence select="(
      $mat[1]*$vec[1] + $mat[4]*$vec[2] + $mat[7]*$vec[3],
      $mat[2]*$vec[1] + $mat[5]*$vec[2] + $mat[8]*$vec[3],
      $mat[3]*$vec[1] + $mat[6]*$vec[2] + $mat[9]*$vec[3])"/>
  </xsl:function>

  <xsl:function name="mat:V3plusV3" as="xs:double*">
    <xsl:param name="vec1" as="xs:double*"/>
    <xsl:param name="vec2" as="xs:double*"/>
    <xsl:sequence select="(
      $vec1[1]+$vec2[1],
      $vec1[2]+$vec2[2],
      $vec1[3]+$vec2[3]
      )"/>
  </xsl:function>

  <xsl:function name="mat:M33timesM33" as="xs:double*">
    <xsl:param name="mat1" as="xs:double*"/>
    <xsl:param name="mat2" as="xs:double*"/>
    <xsl:sequence select="(
      $mat1[1]*$mat2[1] + $mat1[4]*$mat2[2] + $mat1[7]*$mat2[3],
      $mat1[2]*$mat2[1] + $mat1[5]*$mat2[2] + $mat1[8]*$mat2[3],
      $mat1[3]*$mat2[1] + $mat1[6]*$mat2[2] + $mat1[9]*$mat2[3],

      $mat1[1]*$mat2[4] + $mat1[4]*$mat2[5] + $mat1[7]*$mat2[6],
      $mat1[2]*$mat2[4] + $mat1[5]*$mat2[5] + $mat1[8]*$mat2[6],
      $mat1[3]*$mat2[4] + $mat1[6]*$mat2[5] + $mat1[9]*$mat2[6],

      $mat1[1]*$mat2[7] + $mat1[4]*$mat2[8] + $mat1[7]*$mat2[9],
      $mat1[2]*$mat2[7] + $mat1[5]*$mat2[8] + $mat1[8]*$mat2[9],
      $mat1[3]*$mat2[7] + $mat1[6]*$mat2[8] + $mat1[9]*$mat2[9]
      )"/>
  </xsl:function>
  
  <xsl:function name="mat:transposeM33" as="xs:double*">
    <xsl:param name="mat" as="xs:double*"/>
    <xsl:sequence select="(
      $mat[1],
      $mat[4],
      $mat[7],
                  $mat[2],
                  $mat[5],
                  $mat[8],
                              $mat[3],
                              $mat[6],
                              $mat[9]
      )"/>
  </xsl:function>

  <xsl:function name="mat:negate" as="xs:double*">
    <xsl:param name="mat" as="xs:double*"/>
    <xsl:for-each select="$mat">
      <xsl:sequence select="-current()"/>
    </xsl:for-each>
  </xsl:function>

  <xsl:function name="trans:Rx" as="xs:double*">
    <xsl:param name="th" as="xs:double"/>
    <!--We're using column-major order-->
    <xsl:sequence select="(
      1,  
      0,          
      0,
                  0,  
                  math:cos($th), 
                  math:sin($th),
                              0,  
                              -math:sin($th), 
                              math:cos($th) )"/>
  </xsl:function>
  <xsl:function name="trans:Ry" as="xs:double*">
    <xsl:param name="th" as="xs:double"/>
    <!--We're using column-major order-->
    <xsl:sequence select="(
      math:cos($th), 
      0,          
      -math:sin($th),
                  0,  
                  1,  
                  0,
                              math:sin($th), 
                              0,  
                              math:cos($th) )"/>
  </xsl:function>
  <xsl:function name="trans:Rz" as="xs:double*">
    <xsl:param name="th" as="xs:double"/>
    <!--We're using column-major order-->
    <xsl:sequence select="(
      math:cos($th), 
      math:sin($th),
      0,  
                  -math:sin($th), 
                  math:cos($th),
                  0,  
                              0,  
                              0,          
                              1 )"/>
  </xsl:function>

  <xsl:function name="trans:identity" as="xs:double*">
    <!--We're using column-major order-->
    <xsl:sequence select="(
      1,
      0,
      0,
          0,
          1,
          0,
              0,
              0,
              1,
                  0,
                  0,
                  0   )"/>
  </xsl:function>

  <!--<xsl:function name="trans:atan2" as="xs:double">-->
    <!--<xsl:param name="y" as="xs:double"/>-->
    <!--<xsl:param name="x" as="xs:double"/>-->
    <!--<xsl:variable name="eps" select="xs:double(1e-3)"/>-->

    <!--<xsl:variable name="atan_y_div_x" as="xs:double">-->
      <!--<xsl:choose>-->
        <!--<xsl:when test="$x &lt; $eps and $x &gt; -$eps">-->
          <!--<xsl:value-of select="$halfPi"/>-->
        <!--</xsl:when>-->
        <!--<xsl:otherwise>-->
          <!--<xsl:value-of select="math:arctan($y div $x, $eps)"/>-->
          <!--[><xsl:value-of select="$y div $x"/><]-->
        <!--</xsl:otherwise>-->
      <!--</xsl:choose>-->
    <!--</xsl:variable>-->
    <!--<xsl:sequence select="$atan_y_div_x"/>-->
    <!--<xsl:choose>-->
      <!--<xsl:when test="$y &gt; 0 and $atan_y_div_x &lt; 0">-->
        <!--<xsl:sequence select="$atan_y_div_x + $pi"/>-->
      <!--</xsl:when>-->
      <!--<xsl:when test="$y &lt; 0 and $atan_y_div_x &gt; 0">-->
        <!--<xsl:sequence select="$atan_y_div_x - $pi"/>-->
      <!--</xsl:when>-->
      <!--<xsl:otherwise>-->
        <!--<xsl:sequence select="$atan_y_div_x"/>-->
      <!--</xsl:otherwise>-->
    <!--</xsl:choose>-->
  <!--</xsl:function>-->

  <xsl:function name="trans:getP" as="xs:double*">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:sequence select="$T[position()=(10 to 12)]"/>
  </xsl:function>

  <xsl:function name="trans:getR" as="xs:double*">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:sequence select="$T[position()=(1 to 9)]"/>
  </xsl:function>

  <xsl:function name="trans:pose2transform" as="xs:double*">
    <xsl:param name="pose" as="xs:double*"/>
    <xsl:variable name="Rx" select="trans:Rx($pose[4])"/>
    <xsl:variable name="Ry" select="trans:Ry($pose[5])"/>
    <xsl:variable name="Rz" select="trans:Rz($pose[6])"/>
    <xsl:variable name="P"  select="$pose[position()=(1 to 3)]"/>
    <xsl:variable name="R"
      select="mat:M33timesM33($Rz,mat:M33timesM33($Ry,$Rx))"/>
    <xsl:sequence select="($R, $P)"/>
  </xsl:function>

  <xsl:function name="trans:transform2xyz" as="xs:double*">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:sequence select="trans:getP($T)"/>
  </xsl:function>

  <xsl:function name="trans:transform2rpy" as="xs:double*">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:variable name="R" select="trans:getR($T)"/>
    <xsl:variable name="pitch" 
      select="math:atan2(-$R[3], math:sqrt($R[1]*$R[1] + $R[4]*$R[4]))"/>
    <xsl:variable name="roll" 
      select="math:atan2($R[6] div math:cos($pitch), $R[9] div math:cos($pitch))"/>
    <xsl:variable name="yaw" 
      select="math:atan2($R[2] div math:cos($pitch), $R[1] div math:cos($pitch))"/>
    <xsl:sequence select="($roll,$pitch,$yaw)"/>
  </xsl:function>

  <xsl:function name="trans:transform2pose" as="xs:double*">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:variable name="R" select="trans:getR($T)"/>
    <xsl:variable name="P" select="trans:getP($T)"/>
    <xsl:variable name="pitch" 
      select="math:atan2(-$R[3], math:sqrt($R[1]*$R[1] + $R[4]*$R[4]))"/>
    <xsl:variable name="roll" 
      select="math:atan2($R[6] div math:cos($pitch), $R[9] div math:cos($pitch))"/>
    <xsl:variable name="yaw" 
      select="math:atan2($R[2] div math:cos($pitch), $R[1] div math:cos($pitch))"/>
    <xsl:sequence select="($P,$roll,$pitch,$yaw)"/>
  </xsl:function>

  <xsl:function name="trans:inv" as="xs:double*">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:variable name="R" select="trans:getR($T)"/>
    <xsl:variable name="R_inv" select="mat:transposeM33($R)"/>
    <xsl:variable name="P" select="trans:getP($T)"/>
    <xsl:sequence select="($R_inv,mat:negate(mat:M33timesV3($R_inv,$P)))"/>
  </xsl:function>

  <xsl:function name="trans:compound" as="xs:double*">
    <xsl:param name="c_T_b" as="xs:double*"/>
    <xsl:param name="b_T_a" as="xs:double*"/>
    <xsl:variable name="c_R_b" select="trans:getR($c_T_b)"/>
    <xsl:variable name="b_R_a" select="trans:getR($b_T_a)"/>
    <xsl:variable name="c_P_b" select="trans:getP($c_T_b)"/>
    <xsl:variable name="b_P_a" select="trans:getP($b_T_a)"/>

    <xsl:variable name="c_R_a" select="mat:M33timesM33($c_R_b,$b_R_a)"/>
    <xsl:variable name="c_P_a" 
      select="mat:V3plusV3(mat:M33timesV3($c_R_b,$b_P_a),$c_P_b)"/>
    <xsl:sequence select="($c_R_a,$c_P_a)"/>
  </xsl:function>

  <xsl:function name="trans:apply" as="xs:double*">
    <xsl:param name="T" as="xs:double*"/>
    <xsl:param name="r" as="xs:double*"/>
    <xsl:variable name="R" select="trans:getR($T)"/>
    <xsl:variable name="P" select="trans:getP($T)"/>

    <xsl:sequence select="mat:V3plusV3(mat:M33timesV3($R,$r),$P)"/>
  </xsl:function>

</xsl:stylesheet>
