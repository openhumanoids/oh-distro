import os
drc_base_path = os.getenv("DRC_BASE")

import sys
sys.path.append(os.path.join(drc_base_path, "software", "models",
                             "model_transformation"))

import mitUrdfUtils as mit
from jointNameMap import jointNameMap
from lxml import etree
import tempfile
from glob import glob

os.chdir(os.path.dirname(os.path.realpath(__file__)))

meshesDirectory = '../meshes'

original_urdf_path = "../multisense_sl_original.urdf"
urdf_path = "../multisense_sl.urdf"
no_joint_urdf_path = "../multisense_sl_no_joint.urdf"
convex_hull_urdf_path = "../multisense_sl_convex_hull.urdf"
no_collision_urdf_path = "../multisense_sl_no_collision.urdf"

# Convert meshes
for inFile in glob(os.path.join(meshesDirectory, "*.dae")):
    mit.convertMeshTo(inFile, ".obj")
    mit.convertMeshTo(inFile, ".wrl")

for inFile in glob(os.path.join(meshesDirectory, "*.obj")):
    if "chull" not in inFile:
        mit.createConvexHullMesh(inFile)

for inFile in glob(os.path.join(meshesDirectory, "*.wrl")):
    if "chull" not in inFile:
        mit.createConvexHullMesh(inFile)

# Expand all includes to allow us to appropriately change mesh filenames
tmp = tempfile.NamedTemporaryFile()
mit.xacro(original_urdf_path, tmp.name, includes_only=True,
          recursive_includes=True)

# Load urdf
urdf = etree.parse(tmp.name)

# Replace package:// syntax
#mit.replacePackageWithPathInMeshPaths(urdf, "../common_components")

# Use MITDRC joint names and .obj meshes
mit.useObjMeshes(urdf)
mit.renameJoints(urdf, jointNameMap)
urdf.write(urdf_path, pretty_print=True)

# Generate no-joint urdf
mit.weldAllJoints(urdf)
urdf.write(no_joint_urdf_path, pretty_print=True)

# Generate no-joint, no-collision urdf
mit.removeAllCollisions(urdf)
urdf.write(no_collision_urdf_path, pretty_print=True)

# Generate convex hull urdf
mit.addCollisionsFromVisuals(urdf)
mit.useConvexHullMeshes(urdf)
urdf.write(convex_hull_urdf_path, pretty_print=True)
