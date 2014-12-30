import os
import sys
import vtk
import collada
import pdb


def writePolyData(polyDataList, outFile):

    mb = vtk.vtkMultiBlockDataSet()
    mb.SetNumberOfBlocks(len(polyDataList))

    for i, polyData in enumerate(polyDataList):
        print i, type(polyData)
        mb.SetBlock(i, polyData)

    writer = vtk.	vtkXMLMultiBlockDataWriter()
    writer.SetFileName(outFile)
    writer.SetInput(mb)
    writer.Write()


def appendPolyData(polyDataList):
    assert len(polyDataList)
    append = vtk.vtkAppendPolyData()
    for polyData in polyDataList:
        append.AddInput(polyData)
    append.Update()
    return shallowCopy(append.GetOutput())


def transformPolyData(polyData, transform):

    t = vtk.vtkTransformPolyDataFilter()
    t.SetTransform(transform)
    t.SetInput(shallowCopy(polyData))
    t.Update()
    return shallowCopy(t.GetOutput())


def shallowCopy(dataObj):
    newData = dataObj.NewInstance()
    newData.ShallowCopy(dataObj)
    return newData


def createPolyData(faces, vtList, verts, tcoords):

    points = vtk.vtkPoints()
    points.SetDataTypeToDouble()
    points.SetNumberOfPoints(len(vtList))

    tcoordArray = vtk.vtkDoubleArray()
    tcoordArray.SetName('tcoords')
    tcoordArray.SetNumberOfComponents(2)
    tcoordArray.SetNumberOfTuples(len(vtList))

    for i, vt in enumerate(vtList):
        vi, ti = vt
        xyz = verts[vi]
        uv = tcoords[ti]

        points.SetPoint(i, xyz)
        tcoordArray.SetTuple2(i, uv[0], uv[1])

    cells = vtk.vtkCellArray()

    for i, face in enumerate(faces):
        tri = vtk.vtkTriangle()
        tri.GetPointIds().SetId(0, face[0])
        tri.GetPointIds().SetId(1, face[1])
        tri.GetPointIds().SetId(2, face[2])
        cells.InsertNextCell(tri)

    polyData = vtk.vtkPolyData()
    polyData.SetPoints(points)
    polyData.SetPolys(cells)
    polyData.GetPointData().SetTCoords(tcoordArray)
    return polyData


def initMaterialsLibrary(materials):

    global materialsLibrary
    materialsLibrary = {}

    for material in materials:
        materialsLibrary[material.name] = material


def addTextureMetaData(polyData, materialName):

    global materialsLibrary
    material = materialsLibrary[materialName]

    textureFile = material.effect.diffuse.sampler.surface.image.path

    if not os.path.isfile(textureFile):
        print 'warning, cannot find texture file:', os.path.abspath(textureFile)

    s = vtk.vtkStringArray()
    s.InsertNextValue(textureFile)
    s.SetName('texture_filename')
    polyData.GetFieldData().AddArray(s)


def colladaGeometryToPolyData(geometry, transform):

    polyDataList = []

    for primitives in geometry.primitives:
        assert type(primitives) == collada.triangleset.TriangleSet

        polyData = colladaTriangleSetToPolyData(primitives)
        if not polyData:
            continue

        addTextureMetaData(polyData, primitives.material)

        polyData = transformPolyData(polyData, transform)
        polyDataList.append(polyData)

    return polyDataList


def colladaTriangleSetToPolyData(tris):

    if tris.vertex_index is None:
        print 'skipping empty triangle set'
        return None

    nfaces = len(tris.vertex_index)

    if not len(tris.texcoord_indexset):
        print 'skipping triangle set, no texture coords'
        return None

    assert len(tris.texcoord_indexset) == 1
    assert len(tris.texcoord_indexset[0]) == nfaces
    assert len(tris.normal_index) == nfaces


    vertex_index = tris.vertex_index
    normal_index = tris.normal_index
    texcoord_index = tris.texcoord_indexset[0]


    vtList = []
    vtMap = {}
    faces = []

    for triVertInds, triNormalInds, triTexInds, in zip(vertex_index, normal_index, texcoord_index):

        face = []

        for i in xrange(3):

            vi, ni, ti = triVertInds[i], triNormalInds[i], triTexInds[i]
            vt = (vi, ti)

            vtId = vtMap.get(vt)
            if not vtId:
                vtList.append(vt)
                vtId = len(vtList)-1
                vtMap[vt] = vtId
            #else:
            #    print 'found duplicate vertex and texcoord pair'

            face.append(vtId)

        faces.append(face)


    print 'face count:', len(faces)
    print 'vertex count:', len(vtList)

    return createPolyData(faces, vtList, tris.vertex, tris.texcoordset[0])


def colladaTransformToVtk(node):

    assert node.matrix.shape == (4,4)

    m = vtk.vtkMatrix4x4()

    for r in xrange(4):
        for c in xrange(4):
            m.SetElement(r, c, node.matrix[r][c])

    t = vtk.vtkTransform()
    t.SetMatrix(m)
    return t


def colladaNodeToPolyData(node, transform):

    if type(node) == collada.scene.GeometryNode:
        return colladaGeometryToPolyData(node.geometry, transform)

    elif type(node) == collada.scene.Node:

        t = colladaTransformToVtk(node)
        t.PostMultiply()
        t.Concatenate(transform)

        return colladaNodesToPolyData(node.children, t)

    else:
        return []


def colladaNodesToPolyData(nodes, transform):
    return [polyData for node in nodes for polyData in colladaNodeToPolyData(node, transform)]


def colladaSceneToPolyData(scene):
    return colladaNodesToPolyData(scene.nodes, vtk.vtkTransform())


def colladaToPolyData(inFile, outFile):

    print 'reading:', inFile
    f = collada.Collada(inFile, ignore=[collada.DaeUnsupportedError])

    initMaterialsLibrary(f.materials)

    polyDataList = colladaSceneToPolyData(f.scene)

    if not polyDataList:
        return

    print 'writing:', outFile
    writePolyData(polyDataList, outFile)


if __name__ == '__main__':

    if len(sys.argv) < 2:
        print 'Usage: %s <collada files>' % sys.argv[0]
        sys.exit(1)

    for inFile in sys.argv[1:]:
        outFile = os.path.splitext(inFile)[0] + '.vtm'
        colladaToPolyData(inFile, outFile)


