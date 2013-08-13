#!/bin/bash
cat <<END
<?xml version='1.0'?>
<model>
  <name>$1</name>
  <description>$1</description>
  <version>5.0.0</version>
  <sdf>$1.urdf</sdf>
  <author>
    <name>Morgan Quigley</name>
    <email>morgan@osrfoundation.org</email>
  </author>
</model>
END

