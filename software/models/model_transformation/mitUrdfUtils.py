import os
import copy
from lxml import etree
from subprocess import call
from urlparse import urlparse

xacro_path = os.path.join(os.getenv("DRC_BASE"), "software", "models",
                          "model_transformation", "xacro.py")

chull_script_path = os.path.join(os.getenv("DRC_BASE"), "software", "models",
                                 "model_transformation", "chull.mlx")


def convertMeshTo(inFile, newExtension):
    outFile = os.path.splitext(inFile)[0] + newExtension
    convertMesh(inFile, outFile)


def convertMesh(inFile, outFile):
    if not os.path.exists(outFile):
        call(["meshlabserver", "-i", inFile, "-o", outFile])


def createConvexHullMesh(inFile, outFile=None):
    if not outFile:
        inFileBase, inFileExtension = os.path.splitext(inFile)
        outFile = inFileBase + "_chull" + inFileExtension
        if not os.path.exists(outFile):
            call(["meshlabserver", "-i", inFile, "-s", chull_script_path,
                  "-o", outFile])



def removeCollisions(urdf, linkNames):
    for name in linkNames:
        for element in urdf.findall("//link[@name='%s']/collision" % name):
            element.getparent().remove(element)


def removeAllCollisions(urdf):
    for element in urdf.findall("//collision"):
        element.getparent().remove(element)

    return urdf


def addCollisionsFromVisuals(urdf):
    for visual in urdf.findall("//visual"):
        collision = copy.deepcopy(visual)
        collision.tag = "collision"
        visual.getparent().append(collision)


def addFrame(urdf, frameName, linkName, xyz, rpy):
    frame = etree.SubElement(urdf.getroot(), "frame")
    frame.set("name", frameName)
    frame.set("link", linkName)
    frame.set("xyz", xyz)
    frame.set("rpy", rpy)

    return urdf


def addContactPoint(urdf, linkName, xyz, group):
    link = urdf.find("link[@name='%s']" % linkName)

    collision_point = etree.SubElement(link, "collision")
    collision_point.set("group", group)

    origin = etree.SubElement(collision_point, "origin")
    origin.set("rpy", "0 0 0")
    origin.set("xyz", xyz)

    geometry = etree.SubElement(collision_point, "geometry")
    sphere = etree.SubElement(geometry, "sphere")
    sphere.set("radius", "0.0")

    visual_point = etree.SubElement(link, "visual")
    visual_point.set("group", group)

    origin = etree.SubElement(visual_point, "origin")
    origin.set("rpy", "0 0 0")
    origin.set("xyz", xyz)

    geometry = etree.SubElement(visual_point, "geometry")
    sphere = etree.SubElement(geometry, "sphere")
    sphere.set("radius", "0.01")

    return urdf


def xacro(inFile, outFile, includes_only=False, recursive_includes=False,
          verbose=False):
    args = ["python", xacro_path, inFile, "-o", outFile]
    if includes_only:
        args.append("--includes")
    if recursive_includes:
        args.append("--recursive-includes")
    if verbose:
        print("Executing: " + " ".join(args))

    call(args)


def replacePackageWithPathInMeshPaths(urdf, newPath):
    for element in urdf.findall("//*[@filename]"):
        filename = element.get("filename")
        parsed_filename = urlparse(filename)

        if parsed_filename.scheme == "package":
            path = parsed_filename.netloc
            filename = os.path.join(newPath, path + parsed_filename.path)
        else:
            filename = parsed_filename.path

        element.set("filename", filename)


def replaceMeshPaths(urdf, meshDirectory):
    for mesh in urdf.findall(".//mesh"):
        filename = mesh.get("filename")
        newFilename = os.path.join(meshDirectory, os.path.basename(filename))

        mesh.set("filename", newFilename)

    return urdf


def useObjMeshes(urdf):
    for mesh in urdf.findall(".//mesh"):
        filename = mesh.get("filename")
        objFilename = os.path.splitext(filename)[0] + ".obj"
        mesh.set("filename", objFilename)

    return urdf


def useConvexHullMeshes(urdf):
    for mesh in urdf.findall(".//mesh"):
        filename = mesh.get("filename")
        filename_base, filename_ext = os.path.splitext(filename)
        convexHullFilename = filename_base + "_chull" + filename_ext
        mesh.set("filename", convexHullFilename)

    return urdf


def renameJoints(urdf, jointNameMap):

    for oldName, newName in jointNameMap.iteritems():
        for element in urdf.xpath("//*[contains(@name,'%s')]" % oldName):
            element.set("name", element.get("name").replace(oldName,
                                                            newName))

    return urdf


def weldJoint(urdf, jointName):

    joint = urdf.xpath("//joint[@name = '%s']" % jointName)[0]
    joint.set("type", "fixed")


def weldAllJoints(urdf):

    for joint in urdf.findall("//joint"):
        joint.set("type", "fixed")


def addCollisionFilterGroup(urdf, name, members, groupsToIgnore):

    cfg = etree.SubElement(urdf.getroot(), "collision_filter_group")
    cfg.set('name',name)
    for link in members:
        m = etree.SubElement(cfg, 'member')
        m.set('link', link)

    for group in groupsToIgnore:
        g = etree.SubElement(cfg, 'ignored_collision_filter_group')
        g.set('collision_filter_group', group)
