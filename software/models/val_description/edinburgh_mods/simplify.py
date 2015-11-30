import os
import sys
import xml.dom.minidom

# "mesh name": ["link name (optional)"]
# [
#   ["shape type", {"shape param":"value","shape param":"value",...}, {"origin param":"value","origin param":"value",...}, {"material param":"value","material param":"value",...}],
#   ...
# ]
shapes={
  "pelvis.dae": ["",
    [
      ["cylinder",{"length":"0.18","radius":"0.18"}, {"rpy":"0 0 0", "xyz":"0 0 -0.09"},{"name":"White"}], 
      ["sphere",{"radius":"0.18"}, {"xyz":"0 0 -0.18"},{"name":"White"}],
    ],
  ],  
  "torso.dae": ["",
    [
      ["cylinder",{"length":"0.4","radius":"0.18"}, {"rpy":"0 0 0", "xyz":"-0.04 0 0.2"},{"name":"White"}],
      ["cylinder",{"length":"0.3","radius":"0.15"}, {"rpy":"1.57079632679 0 0", "xyz":"-0.005 0 0.23"},{"name":"White"}], 
      ["box",{"size":"0.25 0.36 0.42"}, {"rpy":"0 0 0", "xyz":"-0.17 0 0.20"},{"name":"White"}], 
    ],
  ],  
  "head_multisense_no_visor.dae": ["",
    [
      ["sphere",{"radius":"0.15"}, {"xyz":"0.09 0 0"},{"name":"White"}],
    ],
  ], 
  "aj1_right.dae": ["",
    [
      ["cylinder",{"length":"0.2","radius":"0.08"}, {"rpy":"0 1.57079632679 0", "xyz":"-0.006 -0.246 0"},{"name":"White"}],
    ],
  ],
  "aj1_left.dae": ["",
    [
      ["cylinder",{"length":"0.2","radius":"0.08"}, {"rpy":"0 1.57079632679 0", "xyz":"-0.006 0.246 0"},{"name":"White"}],
    ],
  ],
  "aj2_right.dae": ["",
    [
      ["cylinder",{"length":"0.12","radius":"0.075"}, {"rpy":"1.57079632679 0 0", "xyz":"0 -0.07 0"},{"name":"White"}],
    ],
  ],
  "aj2_left.dae": ["",
    [
      ["cylinder",{"length":"0.12","radius":"0.075"}, {"rpy":"1.57079632679 0 0", "xyz":"0 0.07 0"},{"name":"White"}],
    ],
  ],
  "aj3_right.dae": ["",
    [
      ["cylinder",{"length":"0.20","radius":"0.075"}, {"rpy":"1.57079632679 0 0", "xyz":"0 -0.2 0"},{"name":"White"}],
    ],
  ],
  "aj3_left.dae": ["",
    [
      ["cylinder",{"length":"0.20","radius":"0.075"}, {"rpy":"1.57079632679 0 0", "xyz":"0 0.2 0"},{"name":"White"}],
    ],
  ],
  "aj4_right.dae": ["",
    [
      ["cylinder",{"length":"0.141","radius":"0.061"}, {"rpy":"0 0 0", "xyz":"-0.01 -0.008 0"},{"name":"Gold"}],
    ],
  ],
  "aj4_left.dae": ["",
    [
      ["cylinder",{"length":"0.141","radius":"0.061"}, {"rpy":"0 0 0", "xyz":"-0.01 0.008 0"},{"name":"Gold"}],
    ],
  ],
  "aj5_right.dae": ["",
    [
      ["cylinder",{"length":"0.22","radius":"0.06"}, {"rpy":"1.57079632679 0 0", "xyz":"0 -0.17 0"},{"name":"White"}],
    ],
  ],
  "aj5_left.dae": ["",
    [
      ["cylinder",{"length":"0.22","radius":"0.06"}, {"rpy":"1.57079632679 0 0", "xyz":"0 0.17 0"},{"name":"White"}],
    ],
  ],
  "palm_right.dae": ["",
    [
      ["box",{"size":"0.03 0.12 0.1"}, {"rpy":"0 0 0", "xyz":"-0.0225 -0.048 -0.01"},{"name":"White"}],
      ["box",{"size":"0.025 0.07 0.1"}, {"rpy":"0 0 0", "xyz":"0.005 -0.045 -0.01"},{"name":"White"}],
    ],
  ],
  "palm_left.dae": ["",
    [
      ["box",{"size":"0.03 0.12 0.1"}, {"rpy":"0 0 0", "xyz":"-0.0225 0.048 -0.01"},{"name":"White"}],
      ["box",{"size":"0.025 0.07 0.1"}, {"rpy":"0 0 0", "xyz":"0.005 0.045 -0.01"},{"name":"White"}],
    ],
  ],
  "thumbj2_right.dae": ["",
    [
      ["box",{"size":"0.025 0.015 0.04"}, {"xyz":"0 -0.005 0.025", "rpy":"0.2 0 0"},{"name":"White"}],
    ],
  ],
  "thumbj3_right.dae": ["",
    [
      ["box",{"size":"0.020 0.015 0.03"}, {"xyz":"0 -0.005 0.02", "rpy":"0.2 0 0"},{"name":"White"}],
    ],
  ],
  "thumbj4_right.dae": ["",
    [
      ["box",{"size":"0.017 0.015 0.022"}, {"xyz":"0 -0.005 0.016", "rpy":"0.2 0 0"},{"name":"White"}],
    ],
  ],
  "thumbj2_left.dae": ["",
    [
      ["box",{"size":"0.025 0.015 0.04"}, {"xyz":"0 0.005 0.025", "rpy":"-0.2 0 0"},{"name":"White"}],
    ],
  ],
  "thumbj3_left.dae": ["",
    [
      ["box",{"size":"0.020 0.015 0.03"}, {"xyz":"0 0.005 0.02", "rpy":"-0.2 0 0"},{"name":"White"}],
    ],
  ],
  "thumbj4_left.dae": ["",
    [
      ["box",{"size":"0.017 0.015 0.022"}, {"xyz":"0 0.005 0.016", "rpy":"-0.2 0 0"},{"name":"White"}],
    ],
  ],
  "indexj1_right.dae": ["",
    [
      ["box",{"size":"0.015 0.04 0.025"}, {"rpy":"0 0 0", "xyz":"0 -0.02 0"},{"name":"White"}],
    ],
  ],
  "indexj2_right.dae": ["",
    [
      ["box",{"size":"0.015 0.03 0.020"}, {"rpy":"0 0 0", "xyz":"0 -0.015 0"},{"name":"White"}],
    ],
  ],
  "indexj3_right.dae": ["",
    [
      ["box",{"size":"0.015 0.022 0.017"}, {"rpy":"0 0 0", "xyz":"0 -0.011 0"},{"name":"White"}],
    ],
  ],
  "indexj1_left.dae": ["",
    [
      ["box",{"size":"0.015 0.04 0.025"}, {"rpy":"0 0 0", "xyz":"0 0.02 0"},{"name":"White"}],
    ],
  ],
  "indexj2_left.dae": ["",
    [
      ["box",{"size":"0.015 0.03 0.020"}, {"rpy":"0 0 0", "xyz":"0 0.015 0"},{"name":"White"}],
    ],
  ],
  "indexj3_left.dae": ["",
    [
      ["box",{"size":"0.015 0.022 0.017"}, {"rpy":"0 0 0", "xyz":"0 0.011 0"},{"name":"White"}],
    ],
  ],
  "middlej1_right.dae": ["",
    [
      ["box",{"size":"0.015 0.04 0.025"}, {"rpy":"0 0 0", "xyz":"0 -0.02 0"},{"name":"White"}],
    ],
  ],
  "middlej2_right.dae": ["",
    [
      ["box",{"size":"0.015 0.03 0.020"}, {"rpy":"0 0 0", "xyz":"0 -0.015 0"},{"name":"White"}],
    ],
  ],
  "middlej3_right.dae": ["",
    [
      ["box",{"size":"0.015 0.022 0.017"}, {"rpy":"0 0 0", "xyz":"0 -0.011 0"},{"name":"White"}],
    ],
  ],
  "middlej1_left.dae": ["",
    [
      ["box",{"size":"0.015 0.04 0.025"}, {"rpy":"0 0 0", "xyz":"0 0.02 0"},{"name":"White"}],
    ],
  ],
  "middlej2_left.dae": ["",
    [
      ["box",{"size":"0.015 0.03 0.020"}, {"rpy":"0 0 0", "xyz":"0 0.015 0"},{"name":"White"}],
    ],
  ],
  "middlej3_left.dae": ["",
    [
      ["box",{"size":"0.015 0.022 0.017"}, {"rpy":"0 0 0", "xyz":"0 0.011 0"},{"name":"White"}],
    ],
  ],
  "pinkyj1_right.dae": ["",
    [
      ["box",{"size":"0.015 0.04 0.025"}, {"rpy":"0 0 0", "xyz":"0 -0.02 0"},{"name":"White"}],
    ],
  ],
  "pinkyj2_right.dae": ["",
    [
      ["box",{"size":"0.015 0.03 0.020"}, {"rpy":"0 0 0", "xyz":"0 -0.015 0"},{"name":"White"}],
    ],
  ],
  "pinkyj3_right.dae": ["",
    [
      ["box",{"size":"0.015 0.022 0.017"}, {"rpy":"0 0 0", "xyz":"0 -0.011 0"},{"name":"White"}],
    ],
  ],
  "pinkyj1_left.dae": ["",
    [
      ["box",{"size":"0.015 0.04 0.025"}, {"rpy":"0 0 0", "xyz":"0 0.015 0"},{"name":"White"}],
    ],
  ],
  "pinkyj2_left.dae": ["",
    [
      ["box",{"size":"0.015 0.03 0.020"}, {"rpy":"0 0 0", "xyz":"0 0.015 0"},{"name":"White"}],
    ],
  ],
  "pinkyj3_left.dae": ["",
    [
      ["box",{"size":"0.015 0.022 0.017"}, {"rpy":"0 0 0", "xyz":"0 0.011 0"},{"name":"White"}],
    ],
  ],

  "lj1_right.dae": ["",
    [
      ["sphere",{"radius":"0.14"}, {"xyz":"0 0.01 -0.035"},{"name":"White"}],
    ],
  ],
  "lj1_left.dae": ["",
    [
      ["sphere",{"radius":"0.14"}, {"xyz":"0 0.01 -0.035"},{"name":"White"}],
    ],
  ],
  "lj3_right.dae": ["",
    [
      ["box",{"size":"0.22 0.22 0.29"}, {"rpy":"0 0 0", "xyz":"0.03 -0.07 -0.31"},{"name":"White"}],
      ["box",{"size":"0.22 0.08 0.254"}, {"rpy":"0 0 0", "xyz":"0.03 -0.14 -0.038"},{"name":"White"}],
    ],
  ],
  "lj3_left.dae": ["",
    [
      ["box",{"size":"0.22 0.22 0.29"}, {"rpy":"0 0 0", "xyz":"0.03 0.07 -0.31"},{"name":"White"}],
      ["box",{"size":"0.22 0.08 0.254"}, {"rpy":"0 0 0", "xyz":"0.03 0.14 -0.038"},{"name":"White"}],
    ],
  ],
  "lj4_right.dae": ["",
    [
      ["box",{"size":"0.185 0.18 0.38"}, {"rpy":"0 0 0", "xyz":"-0.008 -0.005 -0.205"},{"name":"White"}],
    ],
  ],
  "lj4_left.dae": ["",
    [
      ["box",{"size":"0.185 0.18 0.38"}, {"rpy":"0 0 0", "xyz":"-0.008  0.005 -0.205"},{"name":"White"}],
    ],
  ],
  "lj6_right.dae": ["",
    [
      ["box",{"size":"0.082 0.128 0.3"}, {"rpy":"0 0.13 0", "xyz":"0.048263578 -3.1591183e-05 0.077849126"},{"name":"White"}],
    ],
  ],
  "lj6_left.dae": ["",
    [
      ["box",{"size":"0.082 0.128 0.3"}, {"rpy":"0 -0.13 0", "xyz":"0.048263578 -3.1591183e-05 -0.077849126"},{"name":"White"}],
    ],
  ],
  "foot.dae": ["",
    [
      ["box",{"size":"0.27 0.16 0.064"}, {"rpy":"0 0 0", "xyz":"0.066 0.0 -0.056"},{"name":"White"}],
    ],
  ],
}

def usage():
    print 'usage:   python simplify.py input.urdf output.urdf' 
    print 'vanilla: python simplify.py input.urdf output.urdf -v'

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

def updateLinks(tree,root,geomtype,vanilla):
    links= root.getElementsByTagName("link")
    if len(links)>0:
      for link in links:
        try:
          cols=link.getElementsByTagName(geomtype)
          if len(cols)>0:
            removed=False
            for col in cols:
              geoms=col.getElementsByTagName("geometry")
              if vanilla and col.hasAttribute("group"):
                col.removeAttribute("group")
              if len(geoms)>0:
                  for geom in geoms:
                    meshes=geom.getElementsByTagName("mesh")
                    if len(meshes)==0:
                      # Remove simple geometry used to model the sensors (small boxes that do not collide anyway)
                      if link.getAttribute("name")=="leftFoot" or link.getAttribute("name")=="rightFoot":
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
    vanilla=False
    if len(sys.argv)>2:
       vanilla=True
    print 'Parsing '+ifile
    DOMTree = xml.dom.minidom.parse(ifile)
    robot = DOMTree.documentElement
    if vanilla:
       remove(robot,"gazebo")
       remove(robot,"actuator")
       remove(robot,"mode")
       remove(robot,"transmission")
       remove(robot,"copSensor")
       remove(robot,"imuSensor")
    updateLinks(DOMTree,robot,"collision",vanilla)
    updateLinks(DOMTree,robot,"visual",vanilla)

    node=addChild(DOMTree,robot,"material",{"name":"White"})
    addChild(DOMTree,node,"color",{"rgba":"1.0 1.0 1.0 1.0"})
    node1=addChild(DOMTree,robot,"material",{"name":"Gold"})
    addChild(DOMTree,node1,"color",{"rgba":"1.0 0.75 0.01 1.0"})

    writer = open(ofile,"wb")
    DOMTree.writexml(writer)
