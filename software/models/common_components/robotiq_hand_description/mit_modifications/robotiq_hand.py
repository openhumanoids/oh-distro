import os
import tempfile
drc_base_path = os.getenv("DRC_BASE")

import sys
sys.path.append(os.path.join(drc_base_path, "software", "models",
                             "model_transformation"))

import mitUrdfUtils as mit
import shutil
from lxml import etree
from glob import glob

os.chdir(os.path.dirname(os.path.realpath(__file__)))

visualMeshesDirectory = "../meshes/s-model/visual"
collisionMeshesDirectory = "../meshes/s-model/collision"
articulatedVisualMeshesDirectory = "../meshes/s-model_articulated/visual"
articulatedCollisionMeshesDirectory = "../meshes/s-model_articulated/collision"

original_urdf_path = "../cfg/robotiq_hand.urdf.xacro"
urdf_path = "../robotiq_hand.xacro"
no_joint_urdf_path = "../robotiq_hand_no_joint.xacro"
no_collision_urdf_path = "../robotiq_hand_no_collision.xacro"

# Convert meshes
for directory in [visualMeshesDirectory,
                  collisionMeshesDirectory,
                  articulatedVisualMeshesDirectory,
                  articulatedCollisionMeshesDirectory]:
    for inFile in glob(os.path.join(directory, "*.stl")):
        mit.convertMeshTo(inFile, ".obj")
        mit.convertMeshTo(inFile, ".wrl")

    for inFile in glob(os.path.join(directory, "*.STL")):
        mit.convertMeshTo(inFile, ".obj")
        mit.convertMeshTo(inFile, ".wrl")

for directory in [collisionMeshesDirectory,
                  articulatedCollisionMeshesDirectory]:
    for inFile in glob(os.path.join(directory, "*.obj")):
        if "chull" not in inFile:
            mit.createConvexHullMesh(inFile)

    for inFile in glob(os.path.join(directory, "*.wrl")):
        if "chull" not in inFile:
            mit.createConvexHullMesh(inFile)

# Expand all includes to ensure proper editing of mesh filenames below
tmp = tempfile.NamedTemporaryFile()
mit.xacro(original_urdf_path, tmp.name, includes_only=True,
          recursive_includes=True)

# Load urdf
urdf = etree.parse(tmp.name)

# Replace package:// syntax
mit.replacePackageWithPathInMeshPaths(urdf, "../common_components/")
# Use .obj meshes
mit.useObjMeshes(urdf)

# Generate full urdf
urdf.write(urdf_path, pretty_print=True)

# Generate no-joint urdf
for joint in urdf.findall("//joint"):
    joint.set("type", "fixed")

urdf.write(no_joint_urdf_path, pretty_print=True)

# Generate no-joint, no-collision urdf
mit.removeAllCollisions(urdf)
urdf.write(no_collision_urdf_path, pretty_print=True)

# Copy over convex-hull hand
shutil.copy("robotiq_hand_convex_hull.xacro", "../")
