import os
import sys
import xml.dom.minidom

# "mesh name": ["link name (optional)"]
# [
#   ["shape type", {"shape param":"value","shape param":"value",...}, {"origin param":"value","origin param":"value",...}, {"material param":"value","material param":"value",...}],
#   ...
# ]
shapes={
  "pelvis.stl": ["",
    [
      ["cylinder",{"length":"0.18","radius":"0.18"}, {"rpy":"0 0 0", "xyz":"0 0 -0.09"},{}], 
      ["sphere",{"radius":"0.18"}, {"xyz":"0 0 -0.18"},{}],
    ],
  ],  
  "wj3.stl": ["",
    [
      ["cylinder",{"length":"0.4","radius":"0.18"}, {"rpy":"0 1.57079632679 0", "xyz":"0.2 0 -0.04"},{}],
      ["cylinder",{"length":"0.3","radius":"0.15"}, {"rpy":"1.57079632679 0 0", "xyz":"0.23 0 -0.005"},{}], 
      ["box",{"size":"0.4 0.36 0.3"}, {"rpy":"0 0 0", "xyz":"0.25 0 -0.2"},{}], 
    ],
  ],  
  "head.stl": ["",
    [
      ["sphere",{"radius":"0.14"}, {"xyz":"-0.09 0 0"},{}],
    ],
  ],  
  "aj1_Right.stl": ["",
    [
      ["cylinder",{"length":"0.2","radius":"0.08"}, {"rpy":"1.57079632679 0 3.14159265359", "xyz":"0 -0.01 -0.246"},{}],
    ],
  ],
  "aj1_Left.stl": ["",
    [
      ["cylinder",{"length":"0.2","radius":"0.08"}, {"rpy":"1.57079632679 0 3.14159265359", "xyz":"0 -0.01 0.246"},{}],
    ],
  ],
  "aj2.stl": ["",
    [
      ["cylinder",{"length":"0.12","radius":"0.075"}, {"rpy":"0 1.57079632679 0", "xyz":"0.07 0 0"},{}],
    ],
  ],
  "aj3_Right.stl": ["",
    [
      ["cylinder",{"length":"0.20","radius":"0.075"}, {"rpy":"0 0 0", "xyz":"0 0 -0.2"},{}],
    ],
  ],
  "aj3_Left.stl": ["",
    [
      ["cylinder",{"length":"0.20","radius":"0.075"}, {"rpy":"0 0 0", "xyz":"0 0 0.2"},{}],
    ],
  ],
  "aj4.stl": ["",
    [
      ["cylinder",{"length":"0.10","radius":"0.06"}, {"rpy":"0 0 0", "xyz":"0.02 -0.015 0"},{}],
    ],
  ],
  "aj5_Right.stl": ["",
    [
      ["cylinder",{"length":"0.22","radius":"0.06"}, {"rpy":"0 0 0", "xyz":"0 0 -0.17"},{}],
    ],
  ],
  "aj5_Left.stl": ["",
    [
      ["cylinder",{"length":"0.22","radius":"0.06"}, {"rpy":"0 0 0", "xyz":"0 0 0.17"},{}],
    ],
  ],
  "palm_Right.stl": ["",
    [
      ["box",{"size":"0.12 0.1 0.03"}, {"rpy":"0 0 0", "xyz":"0.048 0 -0.0225"},{"name":"White"}],
      ["box",{"size":"0.07 0.1 0.025"}, {"rpy":"0 0 0", "xyz":"0.045 0 0.005"},{"name":"White"}],
    ],
  ],
  "palm_Left.stl": ["",
    [
      ["box",{"size":"0.12 0.1 0.03"}, {"rpy":"0 0 0", "xyz":"0.048 0 0.0225"},{"name":"White"}],
      ["box",{"size":"0.07 0.1 0.025"}, {"rpy":"0 0 0", "xyz":"0.045 0 -0.005"},{"name":"White"}],
    ],
  ],
  "tf2.stl": ["",
    [
      ["box",{"size":"0.04 0.015 0.025"}, {"rpy":"0 0 0", "xyz":"0.02 0 0"},{"name":"White"}],
    ],
  ],
  "tf3.stl": ["",
    [
      ["box",{"size":"0.03 0.015 0.020"}, {"rpy":"0 0 0", "xyz":"0.015 0 0"},{"name":"White"}],
    ],
  ],
  "tf4.stl": ["",
    [
      ["box",{"size":"0.022 0.015 0.017"}, {"rpy":"0 0 0", "xyz":"0.011 0 0"},{"name":"White"}],
    ],
  ],
  "pf2.stl": ["",
    [
      ["box",{"size":"0.04 0.015 0.025"}, {"rpy":"0 0 0", "xyz":"0.02 0 0"},{"name":"White"}],
    ],
  ],
  "pf3.stl": ["",
    [
      ["box",{"size":"0.03 0.015 0.020"}, {"rpy":"0 0 0", "xyz":"0.015 0 0"},{"name":"White"}],
    ],
  ],
  "pf4.stl": ["",
    [
      ["box",{"size":"0.022 0.015 0.017"}, {"rpy":"0 0 0", "xyz":"0.011 0 0"},{"name":"White"}],
    ],
  ],
  "sf1.stl": ["",
    [
      ["box",{"size":"0.04 0.015 0.025"}, {"rpy":"0 0 0", "xyz":"0.02 0 0"},{"name":"White"}],
    ],
  ],
  "sf2_ssf3.stl": ["",
    [
      ["box",{"size":"0.03 0.015 0.020"}, {"rpy":"0 0 0", "xyz":"0.015 0 0"},{"name":"White"}],
    ],
  ],
  "sf3_ssf4.stl": ["",
    [
      ["box",{"size":"0.022 0.015 0.017"}, {"rpy":"0 0 0", "xyz":"0.011 0 0"},{"name":"White"}],
    ],
  ],
  "ssf2_Right.stl": ["",
    [
      ["box",{"size":"0.04 0.015 0.025"}, {"rpy":"0 0 1.57079632679", "xyz":"0 0.02 0.05"},{"name":"White"}],
    ],
  ],
  "ssf2_Left.stl": ["",
    [
      ["box",{"size":"0.04 0.015 0.025"}, {"rpy":"0 0 -1.57079632679", "xyz":"0 -0.02 -0.05"},{"name":"White"}],
    ],
  ],
  "sf2_ssf3.stl": ["",
    [
      ["box",{"size":"0.03 0.015 0.020"}, {"rpy":"0 0 0", "xyz":"0.015 0 0"},{"name":"White"}],
    ],
  ],
  "sf3_ssf4.stl": ["",
    [
      ["box",{"size":"0.022 0.015 0.017"}, {"rpy":"0 0 0", "xyz":"0.011 0 0"},{"name":"White"}],
    ],
  ],
  "lj1_Right.stl": ["",
    [
      ["sphere",{"radius":"0.12"}, {"xyz":"0 0.01 -0.035"},{}],
    ],
  ],
  "lj1_Left.stl": ["",
    [
      ["sphere",{"radius":"0.12"}, {"xyz":"0 0.01 0.035"},{}],
    ],
  ],
  "lj3_Right.stl": ["",
    [
      ["box",{"size":"0.35 0.22 0.22"}, {"rpy":"0 0 0", "xyz":"0.28 -0.03 -0.07"},{}],
      ["box",{"size":"0.194 0.22 0.1"}, {"rpy":"0 0 0", "xyz":"0.008 -0.03 -0.13"},{}],
    ],
  ],
  "lj3_Left.stl": ["",
    [
      ["box",{"size":"0.35 0.22 0.22"}, {"rpy":"0 0 0", "xyz":"0.28 -0.03 0.07"},{}],
      ["box",{"size":"0.194 0.22 0.1"}, {"rpy":"0 0 0", "xyz":"0.008 -0.03 0.13"},{}],
    ],
  ],
  "lj4_Right.stl": ["",
    [
      ["box",{"size":"0.36 0.18 0.18"}, {"rpy":"0 0 0", "xyz":"0.22 0.01 0.01"},{}],
    ],
  ],
  "lj4_Left.stl": ["",
    [
      ["box",{"size":"0.36 0.18 0.18"}, {"rpy":"0 0 0", "xyz":"0.22 0.01 -0.01"},{}],
    ],
  ],
  "lj6_Right.stl": ["",
    [
      ["box",{"size":"0.082 0.128 0.3"}, {"rpy":"0 0.13 0", "xyz":"0.048263578 -3.1591183e-05 0.077849126"},{"name":"White"}],
    ],
  ],
  "lj6_Left.stl": ["",
    [
      ["box",{"size":"0.082 0.128 0.3"}, {"rpy":"0 -0.13 0", "xyz":"0.048263578 -3.1591183e-05 -0.077849126"},{"name":"White"}],
    ],
  ],
}

def usage():
    print 'usage: python simplify.py input.urdf output.urdf'

def remove(root,name):
    collection= root.getElementsByTagName(name)
    if len(collection)>0:
      for item in collection:
        try:
          root.removeChild(item)
        except xml.dom.NotFoundErr:
          continue

def addChild(tree,root,name,attrs):
    node=tree.createElement(name)
    for attr, val in attrs.items():
      node.setAttribute(attr,val)
    root.appendChild(node)
    return node

def updateLinks(tree,root,geomtype):
    links= root.getElementsByTagName("link")
    if len(links)>0:
      for link in links:
        try:
          cols=link.getElementsByTagName(geomtype)
          if len(cols)>0:
            removed=False
            for col in cols:
              geoms=col.getElementsByTagName("geometry")
              if col.hasAttribute("group"):
                col.removeAttribute("group")
              if len(geoms)>0:
                  for geom in geoms:
                    meshes=geom.getElementsByTagName("mesh")
                    if len(meshes)==0:
                      # Remove simple geometry used to model the sensors (small boxes that do not collide anyway)
                      if link.getAttribute("name")=="l_foot" or link.getAttribute("name")=="r_foot":
                        addChild(tree,col,"material",{"name":"White"})
                        if geomtype=="collision":
                          continue
                      link.removeChild(col)
                      removed=True
                      break
                    else:
                      for mesh in meshes:
                        filename=mesh.getAttribute("filename")[mesh.getAttribute("filename").rindex("/")+1:]
                        if shapes.has_key(filename):
                          obj=shapes[filename]
                          if len(obj[0])>0 and obj[0]!=link.getAttribute("name"):
                            continue
                          for shape in obj[1]:
                            node=addChild(tree,link,geomtype,{})
                            nodegeom=addChild(tree,node,"geometry",{})
                            addChild(tree,nodegeom,shape[0],shape[1])
                            if len(shape[2])>0:
                              addChild(tree,node,"origin",shape[2])
                            else:
                              origins=col.getElementsByTagName("origin")
                              if len(origins)>0:
                                for origin in origins:
                                  node.appendChild(origin)
                            if len(shape[3])>0:
                              addChild(tree,node,"material",shape[3])
                            else:
                              materials=col.getElementsByTagName("material")
                              if len(materials)>0:
                                for material in materials:
                                  node.appendChild(material)
                      link.removeChild(col)
                      removed=True
                      break

        except xml.dom.NotFoundErr:
          continue

if __name__ == "__main__":
  if len(sys.argv)<2:
    usage()
  else:
    ifile=sys.argv[len(sys.argv)-2]
    ofile=sys.argv[len(sys.argv)-1]
    print 'Parsing '+ifile
    DOMTree = xml.dom.minidom.parse(ifile)
    robot = DOMTree.documentElement
    remove(robot,"gazebo")
    remove(robot,"actuator")
    remove(robot,"mode")
    remove(robot,"transmission")
    remove(robot,"copSensor")
    remove(robot,"imuSensor")
    updateLinks(DOMTree,robot,"collision")
    updateLinks(DOMTree,robot,"visual")

    writer = open(ofile,"wb")
    DOMTree.writexml(writer)
