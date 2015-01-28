#!/usr/bin/python
import os
import rospkg
import shutil
from glob import glob
from urlparse import urlparse
from lxml import etree

os.chdir(os.path.dirname(os.path.realpath(__file__)))

r = rospkg.RosPack()


def copyMeshFiles(urdf, sourceDirectory, destinationDirectory):
    for mesh in urdf.findall(".//mesh"):
        filename = mesh.get("filename")
        parsed_filename = urlparse(filename)

        if parsed_filename.scheme == "package":
            path = r.get_path(parsed_filename.netloc)
            filename = path + parsed_filename.path
        else:
            filename = os.path.join(sourceDirectory, filename)

        new_filename = os.path.join(destinationDirectory,
                                    os.path.basename(filename))
        print('Copying %s to %s' % (filename, new_filename))
        shutil.copy(filename, new_filename)


if __name__ == '__main__':
    sourceDirectory = r.get_path('atlas_description')
    destinationDirectory = os.getcwd()
    osrfOriginalDirectory = os.path.join(destinationDirectory, 'components',
                                         'osrf_original')

    atlasSkeletonSourcePath = os.path.join(sourceDirectory,
                                           'urdf/atlas_v5_simple_shapes.urdf')
    atlasTransmissionSourcePath = os.path.join(sourceDirectory,
                                               'urdf/atlas_v5.transmission')

    atlasSkeletonDestinationPath = os.path.join(osrfOriginalDirectory,
                                                'atlas_v5.urdf')
    atlasTransmissionDestinationPath = os.path.join(osrfOriginalDirectory,
                                                    'atlas_v5_transmission.urdf')

    # Copy urdf files
    if os.path.exists(os.path.join(destinationDirectory, 'components')):
        shutil.rmtree(os.path.join(destinationDirectory, 'components'))
    if not os.path.exists(osrfOriginalDirectory):
        os.makedirs(osrfOriginalDirectory)

    shutil.copy(atlasSkeletonSourcePath, atlasSkeletonDestinationPath)
    shutil.copy(atlasTransmissionSourcePath, atlasTransmissionDestinationPath)

    # Load skeleton urdf
    atlasSkeletonUrdf = etree.parse(atlasSkeletonSourcePath)

    # Copy mesh files
    meshDestinationDirectory = os.path.join(destinationDirectory, 'meshes')
    if os.path.exists(meshDestinationDirectory):
        shutil.rmtree(meshDestinationDirectory)
    if not os.path.exists(meshDestinationDirectory):
        os.makedirs(meshDestinationDirectory)
    copyMeshFiles(atlasSkeletonUrdf, sourceDirectory, meshDestinationDirectory)

    # Copy texture png files
    texturesRelativeDirectory = os.path.join('materials', 'textures')
    texturesSourceDirectory = os.path.join(sourceDirectory,
                                           texturesRelativeDirectory)
    texturesDestinationDirectory = os.path.join(destinationDirectory,
                                                texturesRelativeDirectory)
    if os.path.exists(texturesDestinationDirectory):
        shutil.rmtree(texturesDestinationDirectory)
    if not os.path.exists(texturesDestinationDirectory):
        os.makedirs(texturesDestinationDirectory)

    for inFile in glob(os.path.join(texturesSourceDirectory, '*.png')):
        outFile = os.path.join(texturesDestinationDirectory,
                               os.path.basename(inFile))
        shutil.copy(inFile, outFile)

    for inFile in glob(os.path.join(texturesSourceDirectory, '*.jpg')):
        outFile = os.path.join(texturesDestinationDirectory,
                               os.path.basename(inFile))
        shutil.copy(inFile, outFile)
