#!/usr/bin/python
import os
drc_base_path = os.getenv("DRC_BASE")

import sys
sys.path.append(os.path.join(drc_base_path, "software", "models",
                             "model_transformation"))

import convertCollada
import mitUrdfUtils as mit
import copy
from jointNameMap import jointNameMap
from lxml import etree
from glob import glob
import math


meshesDirectory = "../meshes"

full_mesh_xacro_path = ("../components/osrf_original/atlas_v5.urdf")
full_mesh_urdf_path = ("../components/atlas_v5_full_collision_geometry.urdf")
minimal_contact_urdf_path = "../components/atlas_v5_minimal_contact.urdf"
convex_hull_urdf_path = "../components/atlas_v5_convex_hull.urdf"

# Convert meshes
originalDirectory = os.getcwd()
os.chdir(os.path.abspath(meshesDirectory))
for inFile in glob("*.dae"):
    # Use extremities_diffuse_unplugged_mit.jpg and
    # torso_diffuse_unplugged_mit.jpg
    dae = etree.parse(inFile)

    xpath_str = "//*[text()='../materials/textures/extremities_diffuse_unplugged.jpg']"
    for element in dae.xpath(xpath_str):
        element.text = '../materials/textures/extremities_diffuse_unplugged_mit.jpg'

    xpath_str = "//*[text()='../materials/textures/torso_diffuse_unplugged.jpg']"
    for element in dae.xpath(xpath_str):
        element.text = '../materials/textures/torso_diffuse_unplugged_mit.jpg'

    dae.write(inFile)
    mit.convertMeshTo(inFile, ".obj")
    colladaFile = os.path.splitext(inFile)[0] + '.vtm'
    convertCollada.colladaToPolyData(inFile, colladaFile)
os.chdir(originalDirectory)

for inFile in glob(os.path.join(meshesDirectory, "*.obj")):
    if "chull" not in inFile:
        mit.createConvexHullMesh(inFile)

for inFile in glob(os.path.join(meshesDirectory, "*.obj")):
    mit.convertMeshTo(inFile, ".wrl")

mit.xacro(full_mesh_xacro_path, full_mesh_urdf_path)

urdf = etree.parse(full_mesh_urdf_path)
mit.renameJoints(urdf, jointNameMap)

mit.replaceMeshPaths(urdf, "package://atlas_v5/meshes")
mit.useObjMeshes(urdf)

mit.addFrame(urdf, "l_foot_sole", "l_foot", "0.0426  0.0017 -0.07645", "0 0 0")
mit.addFrame(urdf, "r_foot_sole", "r_foot", "0.0426 -0.0017 -0.07645", "0 0 0")
mit.addFrame(urdf, "l_foot_toe", "l_foot", "0.1728 0.0017 -0.07645", "0 0 0")
mit.addFrame(urdf, "r_foot_toe", "r_foot", "0.1728 -0.0017 -0.07645", "0 0 0")
mit.removeCollisions(urdf, ['mtorso', 'ltorso', 'l_talus', 'r_talus'])

armLinkNames = ['clav', 'scap', 'uarm', 'larm', 'ufarm', 'lfarm', 'hand']
for armLinkName in armLinkNames:
    mit.copyLinkProperties(urdf, 'r_' + armLinkName, 'l_' + armLinkName)

jointCopyExceptions = ['limit', 'safety_controller']
for armJointName in ['arm_shx', 'arm_ely', 'arm_elx', 'arm_uwy', 'arm_mwx']:
    mit.copyJointProperties(urdf, 'r_' + armJointName, 'l_' + armJointName, jointCopyExceptions)
mit.copyJointProperties(urdf, 'r_arm_shz', 'l_arm_shz', jointCopyExceptions + ['origin'])

for jointName in ['arm_shx', 'arm_ely', 'arm_elx', 'arm_uwy', 'arm_lwy']:
    mit.invertJointAxis(urdf, 'l_' + jointName)

mit.setJointOriginRPY(urdf, 'l_arm_shz', [0, 0, math.pi])
mit.setJointOriginRPY(urdf, 'l_arm_uwy', [0, math.pi, 0])
mit.setJointOriginRPY(urdf, 'l_arm_lwy', [0, math.pi, 0])

# update joint limits
# legs
mit.setJointLimits(urdf, 'l_leg_aky', -1.06, 0.72)
mit.setJointLimits(urdf, 'r_leg_aky', -1.06, 0.72)
mit.setJointLimits(urdf, 'l_leg_akx', -0.7, 0.7)
mit.setJointLimits(urdf, 'r_leg_akx', -0.7, 0.7)
mit.setJointLimits(urdf, 'l_leg_hpx', -0.23, 0.52)
mit.setJointLimits(urdf, 'r_leg_hpx', -0.52, 0.23)
mit.setJointLimits(urdf, 'l_leg_hpz', -0.2, 0.82)
mit.setJointLimits(urdf, 'r_leg_hpz', -0.82, 0.2)
# arms
mit.setJointLimits(urdf, 'l_arm_uwy',-2.98, 2.98)
mit.setJointLimits(urdf, 'r_arm_uwy',-2.98, 2.98)
mit.setJointLimits(urdf, 'l_arm_mwx',-1.76, 1.76)
mit.setJointLimits(urdf, 'r_arm_mwx',-1.76, 1.76)
mit.setJointLimits(urdf, 'l_arm_lwy',-2.82, 2.82)
mit.setJointLimits(urdf, 'r_arm_lwy',-2.82, 2.82)
# neck
mit.setJointLimits(urdf, 'neck_ay', -0.605, 1.16)

# Create minimal contact skeleton
mit.removeAllCollisions(urdf)

minimal_contact_urdf = copy.deepcopy(urdf)

contact_pts = {'r_foot': {'heel': ["-0.0876 0.0626 -0.07645", "-0.0876 -0.066 -0.07645"],
                         'toe': ["0.1728 0.0626 -0.07645","0.1728 -0.066 -0.07645"],
                         'midfoot_front': ["0.086 0.0626 -0.07645", "0.086 -0.066 -0.07645"],
                         'midfoot_rear': ["-0.0008 0.0626 -0.07645", "-0.0008 -0.066 -0.07645"]},
               'l_foot': {'heel': ["-0.0876 0.066 -0.07645", "-0.0876 -0.0626 -0.07645"],
                        'toe': ["0.1728 0.066 -0.07645", "0.1728 -0.0626 -0.07645"],
                        'midfoot_front': ["0.086 0.066 -0.07645", "0.086 -0.0626 -0.07645"],
                        'midfoot_rear': ["-0.008 0.066 -0.07645", "-0.008 -0.0626 -0.07645"]},
               'pelvis': {'butt': ["-0.0927 -0.0616 -0.144", "-0.0927 0.0623 -0.144"]}};

for foot_name, groups in contact_pts.iteritems():
    for group_name, pts in groups.iteritems():
        for pt in pts:
            mit.addContactPoint(minimal_contact_urdf, foot_name, pt, group_name)

minimal_contact_urdf.write(minimal_contact_urdf_path, pretty_print=True)

# Create convex hull skeleton
mit.addCollisionsFromVisuals(urdf)

mit.useConvexHullMeshes(urdf)
mit.removeCollisions(urdf, ['mtorso', 'ltorso', 'l_talus', 'r_talus'])

mit.addCollisionFilterGroup(urdf, 'feet', ['l_foot', 'r_foot'], ['feet'])
mit.addCollisionFilterGroup(urdf, 'core', ['utorso', 'pelvis'], ['core'])
mit.addCollisionFilterGroup(urdf, 'ignore_core',
                            ['r_scap', 'l_scap', 'r_clav', 'l_clav'], ['core'])
mit.addCollisionFilterGroup(urdf, 'r_uleg', ['r_uglut', 'r_lglut', 'r_uleg'],
                            ['core', 'r_uleg', 'l_uleg'])
mit.addCollisionFilterGroup(urdf, 'l_uleg', ['l_uglut', 'l_lglut', 'l_uleg'],
                            ['core', 'l_uleg', 'l_uleg'])
mit.addCollisionFilterGroup(urdf, 'r_leg', ['r_lleg', 'r_talus', 'r_foot'],
                            ['r_leg', 'r_uleg'])
mit.addCollisionFilterGroup(urdf, 'l_leg', ['l_lleg', 'l_talus', 'l_foot'],
                            ['l_leg', 'l_uleg'])
mit.addCollisionFilterGroup(urdf, 'r_arm',
                            ['r_clav', 'r_scap', 'r_uarm', 'r_larm', 'r_ufarm',
                             'r_lfarm', 'r_hand'],
                            ['r_arm'])
mit.addCollisionFilterGroup(urdf, 'l_arm',
                            ['l_clav', 'l_scap', 'l_uarm', 'l_larm', 'l_ufarm',
                             'l_lfarm', 'l_hand'],
                            ['l_arm'])

convex_hull_urdf = copy.deepcopy(urdf)

for foot_name, groups in contact_pts.iteritems():
    for group_name, pts in groups.iteritems():
        for pt in pts:
            mit.addContactPoint(convex_hull_urdf, foot_name, pt, group_name)

convex_hull_urdf.write(convex_hull_urdf_path, pretty_print=True)

# Add a geometry to represent the hose for lidar filtering to full collision model
for link in urdf.xpath("//link[contains(@name,'larm')]"):
    collision = mit.addCollision(link)
    mit.addOrigin(collision, xyz=[0.0, 0.15, 0.0])
    geometry = mit.addGeometry(collision)
    mit.addBox(geometry, size=[0.15, 0.20, 0.15])

urdf.write(full_mesh_urdf_path, pretty_print=True)
