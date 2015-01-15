#!/usr/bin/python
import os
drc_base_path = os.getenv("DRC_BASE")

import sys
sys.path.append(os.path.join(drc_base_path, "software", "models",
                             "model_transformation"))

import convertCollada
import mitUrdfUtils as mit
from jointNameMap import jointNameMap
import copy
from lxml import etree
from glob import glob


meshesDirectory = "../meshes"

full_mesh_xacro_path = ("../components/osrf_original/atlas_v5.urdf")
full_mesh_urdf_path = ("../components/atlas_v5_full_collision_geometry.urdf")
minimal_contact_urdf_path = "../components/atlas_v5_minimal_contact.urdf"
convex_hull_urdf_path = "../components/atlas_v5_convex_hull.urdf"

# Convert meshes
originalDirectory = os.getcwd()
os.chdir(os.path.abspath(meshesDirectory))
for inFile in glob("*.dae"):
    mit.convertMeshTo(inFile, ".obj")
    mit.convertMeshTo(inFile, ".wrl")
    colladaFile = os.path.splitext(inFile)[0] + '.vtm'
    convertCollada.colladaToPolyData(inFile, colladaFile)
os.chdir(originalDirectory)

for inFile in glob(os.path.join(meshesDirectory, "*.obj")):
    if "chull" not in inFile:
        mit.createConvexHullMesh(inFile)

for inFile in glob(os.path.join(meshesDirectory, "*.wrl")):
    if "chull" not in inFile:
        mit.createConvexHullMesh(inFile)

mit.xacro(full_mesh_xacro_path, full_mesh_urdf_path)

urdf = etree.parse(full_mesh_urdf_path)
mit.renameJoints(urdf, jointNameMap)

mit.replaceMeshPaths(urdf, "package://atlas_v5/meshes")
mit.useObjMeshes(urdf)

mit.addFrame(urdf, "l_foot_sole", "l_foot", "0.0426  0.0017 -0.07645", "0 0 0")
mit.addFrame(urdf, "r_foot_sole", "r_foot", "0.0426 -0.0017 -0.07645", "0 0 0")
mit.removeCollisions(urdf, ['mtorso', 'ltorso', 'l_talus', 'r_talus'])

urdf.write(full_mesh_urdf_path, pretty_print=True)

# Create convex hull skeleton
convex_hull_urdf = copy.deepcopy(urdf)
mit.useConvexHullMeshes(convex_hull_urdf)

mit.addContactPoint(convex_hull_urdf, "r_foot", "-0.0876 0.0626 -0.07645",
                    "heel")
mit.addContactPoint(convex_hull_urdf, "r_foot", "-0.0876 -0.066 -0.07645",
                    "heel")
mit.addContactPoint(convex_hull_urdf, "r_foot", "0.1728 0.0626 -0.07645",
                    "toe")
mit.addContactPoint(convex_hull_urdf, "r_foot", "0.1728 -0.066 -0.07645",
                    "toe")

mit.addContactPoint(convex_hull_urdf, "l_foot", "-0.0876 0.066 -0.07645",
                    "heel")
mit.addContactPoint(convex_hull_urdf, "l_foot", "-0.0876 -0.0626 -0.07645",
                    "heel")
mit.addContactPoint(convex_hull_urdf, "l_foot", "0.1728 0.066 -0.07645",
                    "toe")
mit.addContactPoint(convex_hull_urdf, "l_foot", "0.1728 -0.0626 -0.07645",
                    "toe")

mit.addCollisionFilterGroup(convex_hull_urdf, 'feet', ['l_foot', 'r_foot'],
                            ['feet'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'core', ['utorso', 'pelvis'],
                            ['core'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'ignore_core',
                            ['r_scap', 'l_scap', 'r_clav', 'l_clav'],
                            ['core'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'r_uleg',
                            ['r_uglut', 'r_lglut', 'r_uleg'],
                            ['core', 'r_uleg', 'l_uleg'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'l_uleg',
                            ['l_uglut', 'l_lglut', 'l_uleg'],
                            ['core', 'l_uleg', 'l_uleg'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'r_leg',
                            ['r_lleg', 'r_talus', 'r_foot'],
                            ['r_leg', 'r_uleg'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'l_leg',
                            ['l_lleg', 'l_talus', 'l_foot'],
                            ['l_leg', 'l_uleg'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'r_arm',
                            ['r_uarm', 'r_larm', 'r_ufarm', 'r_lfarm',
                             'r_hand'],
                            ['r_arm'])
mit.addCollisionFilterGroup(convex_hull_urdf, 'l_arm',
                            ['l_uarm', 'l_larm', 'l_ufarm', 'l_lfarm',
                             'l_hand'],
                            ['l_arm'])

convex_hull_urdf.write(convex_hull_urdf_path, pretty_print=True)

# Create minimal contact skeleton
mit.removeAllCollisions(urdf)

mit.addContactPoint(urdf, "r_foot", "-0.0876 0.0626 -0.07645", "heel")
mit.addContactPoint(urdf, "r_foot", "-0.0876 -0.066 -0.07645", "heel")
mit.addContactPoint(urdf, "r_foot", "0.1728 0.0626 -0.07645", "toe")
mit.addContactPoint(urdf, "r_foot", "0.1728 -0.066 -0.07645", "toe")

mit.addContactPoint(urdf, "l_foot", "-0.0876 0.066 -0.07645", "heel")
mit.addContactPoint(urdf, "l_foot", "-0.0876 -0.0626 -0.07645", "heel")
mit.addContactPoint(urdf, "l_foot", "0.1728 0.066 -0.07645", "toe")
mit.addContactPoint(urdf, "l_foot", "0.1728 -0.0626 -0.07645", "toe")

urdf.write(minimal_contact_urdf_path, pretty_print=True)
