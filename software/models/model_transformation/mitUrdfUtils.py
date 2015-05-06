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


def addVisual(link):
    return etree.SubElement(link, 'visual')


def addCollision(link):
    return etree.SubElement(link, 'collision')


def addGeometry(element):
    return etree.SubElement(element, 'geometry')


def addOrigin(element, xyz=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0]):
    origin = etree.SubElement(element, 'origin')
    origin.set('xyz', '%8.5f %8.5f %8.5f' % tuple(xyz))
    origin.set('rpy', '%8.5f %8.5f %8.5f' % tuple(rpy))
    return origin


def addBox(geometry, size=[1.0, 1.0, 1.0]):
    box = etree.SubElement(geometry, 'box')
    box.set('size', '%8.5f %8.5f %8.5f' % tuple(size))
    return box

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
    for mesh in urdf.findall(".//collision/geometry/mesh"):
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

def copyLinkProperties(urdf, sourceLinkName, destinationLinkName):
    sourceLink = urdf.find("link[@name='%s']" % sourceLinkName)
    destinationLink = urdf.find("link[@name='%s']" % destinationLinkName)
    return copyElementProperties(urdf, sourceLink, destinationLink)

def copyJointProperties(urdf, sourceJointName, destinationJointName, additionalExceptions = []):
    sourceJoint = urdf.find("joint[@name='%s']" % sourceJointName)
    destinationJoint = urdf.find("joint[@name='%s']" % destinationJointName)
    return copyElementProperties(urdf, sourceJoint, destinationJoint, ['parent', 'child'] + additionalExceptions)

def copyElementProperties(urdf, sourceElement, destinationElement, exceptionTagNames = []):
    '''
    doing it this way to try to preserve the order of the elements for easy text comparison
    '''
    sourceChildrenToAppend = copy.copy(list(sourceElement))
    for destinationChild in destinationElement:
        if destinationChild.tag not in exceptionTagNames:
            sourceChildrenWithThisTag = sourceElement.findall(destinationChild.tag)
            destinationChildrenWithThisTag = destinationElement.findall(destinationChild.tag)

            if len(sourceChildrenWithThisTag) == 1 and len(destinationChildrenWithThisTag) == 1:
                # replace and remove from sourceChildrenToAppend
                sourceChild = sourceChildrenWithThisTag[0]
                destinationElement.replace(destinationChild, copy.deepcopy(sourceChild))
                sourceChildrenToAppend.remove(sourceChild)
            else:
                # remove and leave in sourceChildrenToAppend
                destinationElement.remove(destinationChild)
    for sourceChild in sourceChildrenToAppend:
        if sourceChild.tag not in exceptionTagNames:
            destinationElement.append(copy.deepcopy(sourceChild))

    return urdf

def invertJointAxis(urdf, jointName):
    axis = urdf.find("joint[@name='%s']/axis" % jointName)
    xyz = axis.get('xyz').split(' ')
    axis.set('xyz', ' '.join(map(lambda x : str(-float(x)), xyz)))

    return urdf

def setJointOriginRPY(urdf, jointName, rpy):
    origin = urdf.find("joint[@name='%s']/origin" % jointName)
    origin.set('rpy', ' '.join(map(lambda x : str(x), rpy)))

    return urdf

def setJointLimits(urdf, jointName, lower, upper):
    origin = urdf.find("joint[@name='%s']/limit" % jointName)
    origin.set('lower', str(lower))
    origin.set('upper', str(upper))

    return urdf

def setLinkVisualRPY(urdf, linkName, rpy):
    visual = urdf.find("link[@name='%s']/visual" % linkName)
    origin = visual.get('origin')
    if origin is None:
        origin = etree.SubElement(visual, 'origin', {'rpy': "0 0 0", 'xyz': "0 0 0"})
    origin.set('rpy', ' '.join(map(lambda x : str(x), rpy)))

    return urdf
