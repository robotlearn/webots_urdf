"""URDF parser"""

import os
import sys
import struct
import math_utils
import gazebo_materials
import numpy
try:
    from PIL import Image
except ImportError as e:
    if sys.platform == 'linux2':
        sys.stderr.write("PIL module not found, please install it with:\n")
        sys.stderr.write("apt-get install python-pip\n")
        sys.stderr.write("pip install pillow\n")
    raise e

from collada import Collada  # for .dae files
import numbers


counter = 0
robotName = ''  # to pass from external


class Trimesh(object):
    """Define triangular mesh object."""

    def __init__(self):
        """Initialization."""
        self.coord = []  # list of coordinate points
        self.coordIndex = []  # list of index of points
        self.texCoord = []  # list of coordinate points for texture
        self.texCoordIndex = []  # list of index for texture
        self.normal = []  # list of normals
        self.normalIndex = []  # list of index of normals


class Inertia(object):
    """Define inertia object."""

    def __init__(self):
        """Initialization."""
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.mass = 1.0
        self.ixx = 1.0
        self.ixy = 0.0
        self.ixz = 0.0
        self.iyy = 1.0
        self.iyz = 0.0
        self.izz = 1.0


class Box(object):
    """Define box object."""

    def __init__(self):
        """Initialization."""
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Cylinder(object):
    """Define cylinder object."""

    def __init__(self):
        """Initialization."""
        self.radius = 0.0
        self.length = 0.0


class Sphere(object):
    """Define sphere object."""

    def __init__(self):
        """Initialization."""
        self.radius = 0.0


class Geometry(object):
    """Define geometry object."""

    def __init__(self):
        """Initialization."""
        self.box = Box()
        self.cylinder = Cylinder()
        self.sphere = Sphere()
        self.trimesh = Trimesh()
        self.scale = [1.0, 1.0, 1.0]


class Color(object):
    """Define color object."""

    def __init__(self, red=0.5, green=0.0, blue=0.0, alpha=1.0):
        """Initialization."""
        self.red = red
        self.green = green
        self.blue = blue
        self.alpha = alpha


class Material(object):
    """Define material object."""

    namedMaterial = {}

    def __init__(self):
        """Initialization."""
        self.emission = Color(0.0, 0.0, 0.0, 1.0)
        self.ambient = Color(0.0, 0.0, 0.0, 0.0)
        self.diffuse = Color(0.5, 0.5, 0.5, 1.0)
        self.specular = Color(0.0, 0.0, 0.0, 1.0)
        self.shininess = None
        self.index_of_refraction = 1.0
        self.texture = ""

    def parseFromMaterialNode(self, node):
        """Parse a material node."""
        if hasElement(node, 'color'):
            colorElement = node.getElementsByTagName('color')[0]
            colors = colorElement.getAttribute('rgba').split()
            self.diffuse.r = float(colors[0])
            self.diffuse.g = float(colors[1])
            self.diffuse.b = float(colors[2])
            self.diffuse.alpha = float(colors[3])
        if node.hasAttribute('name') and node.getAttribute('name') not in Material.namedMaterial:
            Material.namedMaterial[node.getAttribute('name')] = self


class Visual(object):
    """Define visual object."""

    def __init__(self):
        """Initialization."""
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.geometry = Geometry()
        self.material = Material()


class Collision(object):
    """Define collision object."""

    def __init__(self):
        """Initialization."""
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.geometry = Geometry()


class Calibration(object):
    """Define calibration object."""

    def __init__(self):
        """Initialization."""
        self.limit = 0.0
        self.rising = True


class Dynamics(object):
    """Define dynamics object."""

    def __init__(self):
        """Initialization."""
        self.damping = 0.0
        self.friction = 0.0


class Limit(object):
    """Define joint limit object."""

    def __init__(self):
        """Initialization."""
        self.lower = 0.0
        self.upper = 0.0
        self.effort = 0.0
        self.velocity = 0.0


class Safety(object):
    """Define joint safety object."""

    def __init__(self):
        """Initialization."""
        self.lower = 0.0
        self.upper = 0.0
        self.kPosition = 0.0
        self.kVelocity = 0.0


class Link(object):
    """Define link object."""

    def __init__(self):
        """Initialization."""
        self.name = 'default'
        self.inertia = Inertia()
        self.visual = []
        self.collision = []


class Joint(object):
    """Define joint object."""

    def __init__(self):
        """Initialization."""
        self.name = 'default'
        self.type = 'default'
        self.position = [0.0, 0.0, 0.0]
        self.rotation = [1.0, 0.0, 0.0, 0.0]
        self.parent = 'default'
        self.child = 'default'
        self.axis = []
        self.calibration = Calibration()
        self.dynamics = Dynamics()
        self.limit = Limit()
        self.safety = Safety()


class IMU(object):
    """Define an IMU sensor."""

    list = []

    def __init__(self):
        """Initialization."""
        self.name = 'imu'
        self.gaussianNoise = 0
        self.parentLink = None

    def export(self, file, indentationLevel):
        """Export this IMU."""
        indent = '  '
        file.write(indentationLevel * indent + 'Accelerometer {\n')
        file.write(indentationLevel * indent + '  name "%s accelerometer"\n' % self.name)
        if self.gaussianNoise > 0:
            file.write(indentationLevel * indent + '  lookupTable [-100 -100 %lf, 100 100 %lf]\n' % (-self.gaussianNoise / 100.0, self.gaussianNoise / 100.0))
        file.write(indentationLevel * indent + '}\n')
        file.write(indentationLevel * indent + 'Gyro {\n')
        file.write(indentationLevel * indent + '  name "%s gyro"\n' % self.name)
        if self.gaussianNoise > 0:
            file.write(indentationLevel * indent + '  lookupTable [-100 -100 %lf, 100 100 %lf]\n' % (-self.gaussianNoise / 100.0, self.gaussianNoise / 100.0))
        file.write(indentationLevel * indent + '}\n')
        file.write(indentationLevel * indent + 'Compass {\n')
        file.write(indentationLevel * indent + '  name "%s compass"\n' % self.name)
        if self.gaussianNoise > 0:
            file.write(indentationLevel * indent + '  lookupTable [-1 -1 %lf, 1 1 %lf]\n' % -self.gaussianNoise, self.gaussianNoise)
        file.write(indentationLevel * indent + '}\n')


class Camera(object):
    """Define a camera sensor."""

    list = []

    def __init__(self):
        """Initialization."""
        self.name = 'camera'
        self.fov = None
        self.width = None
        self.height = None
        self.noise = None

    def export(self, file, indentationLevel):
        """Export this camera."""
        indent = '  '
        file.write(indentationLevel * indent + 'Camera {\n')
        file.write(indentationLevel * indent + '  name "%s"\n' % self.name)
        if self.fov:
            file.write(indentationLevel * indent + '  fieldOfView %lf\n' % self.fov)
        if self.width:
            file.write(indentationLevel * indent + '  width %d\n' % self.width)
        if self.height:
            file.write(indentationLevel * indent + '  height %d\n' % self.height)
        if self.noise:
            file.write(indentationLevel * indent + '  noise %lf\n' % self.noise)
        file.write(indentationLevel * indent + '}\n')


class Lidar(object):
    """Define a lidar sensor."""

    list = []

    def __init__(self):
        """Initialization."""
        self.name = 'lidar'
        self.fov = None
        self.verticalFieldOfView = None
        self.horizontalResolution = None
        self.numberOfLayers = 1
        self.minRange = None
        self.maxRange = None
        self.resolution = None
        self.noise = None

    def export(self, file, indentationLevel):
        """Export this camera."""
        indent = '  '
        file.write(indentationLevel * indent + 'Lidar {\n')
        file.write(indentationLevel * indent + '  name "%s"\n' % self.name)
        if self.fov:
            file.write(indentationLevel * indent + '  fieldOfView %lf\n' % self.fov)
        if self.verticalFieldOfView:
            file.write(indentationLevel * indent + '  verticalFieldOfView %lf\n' % self.verticalFieldOfView)
        if self.horizontalResolution:
            file.write(indentationLevel * indent + '  horizontalResolution %d\n' % self.horizontalResolution)
        if self.numberOfLayers:
            file.write(indentationLevel * indent + '  numberOfLayers %d\n' % self.numberOfLayers)
        if self.minRange:
            if self.minRange < 0.01:
                file.write(indentationLevel * indent + '  near %lf\n' % self.minRange)
            file.write(indentationLevel * indent + '  minRange %lf\n' % self.minRange)
        if self.maxRange:
            file.write(indentationLevel * indent + '  maxRange %lf\n' % self.maxRange)
        if self.noise:
            file.write(indentationLevel * indent + '  noise %lf\n' % self.noise)
        if self.resolution:
            file.write(indentationLevel * indent + '  resolution %lf\n' % self.resolution)
        file.write(indentationLevel * indent + '}\n')


def colorVector2Instance(cv, alpha_last=True):
    """Eval color object from a vector."""
    c = Color()
    if alpha_last:
        c.red = cv[0]
        c.green = cv[1]
        c.blue = cv[2]
        c.alpha = cv[3]
    else:
        c.red = cv[1]
        c.green = cv[2]
        c.blue = cv[3]
        c.alpha = cv[0]

    return c


def getRobotName(node):
    """Parse robot name.

    Args:
        node (DOM): XML node.
    """
    name = node.getAttribute('name')
    print('Robot name: ' + name)
    return name


def hasElement(node, element):
    """Check if exlement existing in a tag.

    Args:
        node (DOM): XML node.
        element (str): tag name.
    """
    if node.getElementsByTagName(element).length > 0:
        return True
    else:
        return False


def getSTLMesh(filename, node):
    """Read stl file.

    Args:
        filename (str): path to the STL file.
        node (Visual): visual instance.
    """
    stlFile = open(filename, 'rb')
    stlFile.read(80)  # read first 80 characters

    vertex1, vertex2, vertex3 = [], [], []
    numTriangles = struct.unpack("@i", stlFile.read(4))[0]

    struct.unpack("<3f", stlFile.read(12))
    a = struct.unpack("<3f", stlFile.read(12))
    vertex1.append(a)
    b = struct.unpack("<3f", stlFile.read(12))
    vertex2.append(b)
    c = struct.unpack("<3f", stlFile.read(12))
    vertex3.append(c)
    struct.unpack("H", stlFile.read(2))

    # get reference to trimesh object and first coordinates
    trimesh = node.geometry.trimesh
    trimesh.coord.append(vertex1[0])
    trimesh.coord.append(vertex2[0])
    trimesh.coord.append(vertex3[0])

    # for each triangle
    for i in range(1, numTriangles):

        # add each vertex coordinate
        struct.unpack("<3f", stlFile.read(12))
        a = struct.unpack("<3f", stlFile.read(12))
        vertex1.append(a)
        if trimesh.coord.count(a) == 0:  # if no occurence of 'a', add it to the coordinates
            trimesh.coord.append(a)
        b = struct.unpack("<3f", stlFile.read(12))
        vertex2.append(b)
        if trimesh.coord.count(b) == 0:
            trimesh.coord.append(b)
        c = struct.unpack("<3f", stlFile.read(12))
        vertex3.append(c)
        if trimesh.coord.count(c) == 0:
            trimesh.coord.append(c)
        struct.unpack("H", stlFile.read(2))

        # add coordinate indices
        trimesh.coordIndex.append([trimesh.coord.index(vertex1[i]),
                                   trimesh.coord.index(vertex2[i]),
                                   trimesh.coord.index(vertex3[i])])
    stlFile.close()
    return node


def getColladaMesh(filename, node, link):
    """Read collada file.

    Args:
        filename (str): path to the Collada (.dae) file.
        node (DOM): XML node.
        link (Link): link instance.
    """
    colladaMesh = Collada(filename)
    if hasattr(node, 'material') and node.material:
        for geometry in list(colladaMesh.scene.objects('geometry')):
            for data in list(geometry.primitives()):
                visual = Visual()
                visual.position = node.position
                visual.rotation = node.rotation
                visual.material.diffuse.red = node.material.diffuse.red
                visual.material.diffuse.green = node.material.diffuse.green
                visual.material.diffuse.blue = node.material.diffuse.blue
                visual.material.diffuse.alpha = node.material.diffuse.alpha
                visual.material.texture = node.material.texture
                visual.geometry.scale = node.geometry.scale
                for val in data.vertex:
                    visual.geometry.trimesh.coord.append(numpy.array(val))
                for val in data.vertex_index:
                    visual.geometry.trimesh.coordIndex.append(val)
                if data.texcoordset:  # non-empty
                    for val in data.texcoordset[0]:
                        visual.geometry.trimesh.texCoord.append(val)
                if data.texcoord_indexset:  # non-empty
                    for val in data.texcoord_indexset[0]:
                        visual.geometry.trimesh.texCoordIndex.append(val)
                if hasattr(data, '_normal') and data._normal is not None and data._normal.size > 0:
                    for val in data._normal:
                        visual.geometry.trimesh.normal.append(numpy.array(val))
                    if hasattr(data, '_normal_index') and data._normal_index is not None and data._normal_index.size > 0:
                        for val in data._normal_index:
                            visual.geometry.trimesh.normalIndex.append(val)
                if data.material and data.material.effect:
                    if data.material.effect.emission:
                        visual.material.emission = colorVector2Instance(data.material.effect.emission)
                    if data.material.effect.ambient:
                        visual.material.ambient = colorVector2Instance(data.material.effect.ambient)
                    if data.material.effect.specular:
                        visual.material.specular = colorVector2Instance(data.material.effect.specular)
                    if data.material.effect.shininess:
                        visual.material.shininess = data.material.effect.shininess
                    if data.material.effect.index_of_refraction:
                        visual.material.index_of_refraction = data.material.effect.index_of_refraction
                    if data.material.effect.diffuse:
                        if numpy.size(data.material.effect.diffuse) > 1\
                                and all([isinstance(x, numbers.Number) for x in data.material.effect.diffuse]):
                            # diffuse is defined by values
                            visual.material.diffuse = colorVector2Instance(data.material.effect.diffuse)
                        else:
                            # diffuse is defined by *.tif files
                            visual.material.texture = 'textures/' + data.material.effect.diffuse.sampler.surface.image.path.split('/')[-1]
                            txt = os.path.splitext(visual.material.texture)[1]
                            if txt == '.tiff' or txt == '.tif':
                                for dirname, dirnames, filenames in os.walk('.'):
                                    for file in filenames:
                                        if file == str(visual.material.texture.split('/')[-1]):
                                            try:
                                                tifImage = Image.open(os.path.join(dirname, file))
                                                img = './' + robotName + '_textures'
                                                tifImage.save(os.path.splitext(os.path.join(img, file))[0] + '.png')
                                                visual.material.texture = robotName + '_textures/' + os.path.splitext(file)[0] + '.png'
                                                print('translated image ' + visual.material.texture)
                                            except IOError:
                                                visual.material.texture = ""
                                                print('failed to open ' + os.path.join(dirname, file))
                link.visual.append(visual)
    else:
        for geometry in list(colladaMesh.scene.objects('geometry')):
            for data in list(geometry.primitives()):
                collision = Collision()
                collision.position = node.position
                collision.rotation = node.rotation
                collision.geometry.scale = node.geometry.scale
                for value in data.vertex:
                    collision.geometry.trimesh.coord.append(numpy.array(value))
                for value in data.vertex_index:
                    collision.geometry.trimesh.coordIndex.append(value)
                link.collision.append(collision)


def getPosition(node):
    """Read position of a phsical or visual object.

    Args:
        node (DOM): XML node.
    """
    position = [0.0, 0.0, 0.0]
    positionString = node.getElementsByTagName('origin')[0].getAttribute('xyz').split()
    position[0] = float(positionString[0])
    position[1] = float(positionString[1])
    position[2] = float(positionString[2])
    return position


def getRotation(node, isCylinder=False):
    """Read rotation of a phsical or visual object.

    Args:
        node (DOM): XML node.
        isCylinder (bool): if True, we have a cylinder.
    """
    rotation = [0.0, 0.0, 0.0]
    if hasElement(node, 'origin'):
        orientationString = node.getElementsByTagName('origin')[0].getAttribute('rpy').split()
        rotation[0] = float(orientationString[0])
        rotation[1] = float(orientationString[1])
        rotation[2] = float(orientationString[2])
    if isCylinder:
        return math_utils.convertRPYtoEulerAxis(rotation, True)
    else:
        return math_utils.convertRPYtoEulerAxis(rotation, False)


def getInertia(node):
    """Parse inertia of a link.

    Args:
        node (DOM): XML node.
    """
    inertia = Inertia()
    inertialElement = node.getElementsByTagName('inertial')[0]
    if hasElement(inertialElement, 'origin'):
        if inertialElement.getElementsByTagName('origin')[0].getAttribute('xyz'):
            inertia.position = getPosition(inertialElement)
        if inertialElement.getElementsByTagName('origin')[0].getAttribute('rpy'):
            inertia.rotation = getRotation(inertialElement)
    if hasElement(inertialElement, 'mass'):
        inertia.mass = float(inertialElement.getElementsByTagName('mass')[0].getAttribute('value'))
    if hasElement(inertialElement, 'inertia'):
        matrixNode = inertialElement.getElementsByTagName('inertia')[0]
        inertia.ixx = float(matrixNode.getAttribute('ixx'))
        inertia.ixy = float(matrixNode.getAttribute('ixy'))
        inertia.ixz = float(matrixNode.getAttribute('ixz'))
        inertia.iyy = float(matrixNode.getAttribute('iyy'))
        inertia.iyz = float(matrixNode.getAttribute('iyz'))
        inertia.izz = float(matrixNode.getAttribute('izz'))
    return inertia


def getVisual(link, node):
    """Parse visual data of a link.

    Args:
        link (Link): link object.
        node (DOM): xml node.
    """

    for index in range(0, len(node.getElementsByTagName('visual'))):

        # create visual object
        visual = Visual()
        visualElement = node.getElementsByTagName('visual')[index]

        if hasElement(visualElement, 'origin'):
            if visualElement.getElementsByTagName('origin')[0].getAttribute('xyz'):
                visual.position = getPosition(visualElement)
            if visualElement.getElementsByTagName('origin')[0].getAttribute('rpy'):
                if hasElement(visualElement.getElementsByTagName('geometry')[0], 'cylinder'):
                    visual.rotation = getRotation(visualElement, True)
                else:
                    visual.rotation = getRotation(visualElement)
        elif hasElement(visualElement.getElementsByTagName('geometry')[0], 'cylinder'):
            visual.rotation = getRotation(visualElement, True)

        geometryElement = visualElement.getElementsByTagName('geometry')[0]

        if hasElement(visualElement, 'material'):
            material = visualElement.getElementsByTagName('material')[0]
            if hasElement(material, 'color'):
                colorElement = material.getElementsByTagName('color')[0].getAttribute('rgba').split()
                visual.material.diffuse.red = float(colorElement[0])
                visual.material.diffuse.green = float(colorElement[1])
                visual.material.diffuse.blue = float(colorElement[2])
                visual.material.diffuse.alpha = float(colorElement[3])
            elif material.hasAttribute('name') and material.getAttribute('name') in Material.namedMaterial:
                visual.material = Material.namedMaterial[material.getAttribute('name')]
            elif material.firstChild and material.firstChild.nodeValue in gazebo_materials.materials:
                materialName = material.firstChild.nodeValue
                visual.material.diffuse.red = float(gazebo_materials.materials[materialName]['diffuse'][0])
                visual.material.diffuse.green = float(gazebo_materials.materials[materialName]['diffuse'][1])
                visual.material.diffuse.blue = float(gazebo_materials.materials[materialName]['diffuse'][2])
                visual.material.diffuse.alpha = float(gazebo_materials.materials[materialName]['diffuse'][3])
                visual.material.ambient.red = float(gazebo_materials.materials[materialName]['ambient'][0])
                visual.material.ambient.green = float(gazebo_materials.materials[materialName]['ambient'][1])
                visual.material.ambient.blue = float(gazebo_materials.materials[materialName]['ambient'][2])
                visual.material.ambient.alpha = float(gazebo_materials.materials[materialName]['ambient'][3])
                visual.material.specular.red = float(gazebo_materials.materials[materialName]['specular'][0])
                visual.material.specular.green = float(gazebo_materials.materials[materialName]['specular'][1])
                visual.material.specular.blue = float(gazebo_materials.materials[materialName]['specular'][2])
                visual.material.specular.alpha = float(gazebo_materials.materials[materialName]['specular'][3])
            if hasElement(material, 'texture'):
                visual.material.texture = material.getElementsByTagName('texture')[0].getAttribute('filename')
                if os.path.splitext(visual.material.texture)[1] == '.tiff' or os.path.splitext(visual.material.texture)[1] == '.tif':
                    for dirname, dirnames, filenames in os.walk('.'):
                        for filename in filenames:
                            if filename == str(visual.material.texture.split('/')[-1]):
                                print('try to translate image ' + filename)
                                try:
                                    tifImage = Image.open(os.path.join(dirname, filename))
                                    tifImage.save(os.path.splitext(os.path.join('./' + robotName + '_' + 'textures', filename))[0] + '.png')
                                    visual.material.texture = robotName + '_' + 'textures/' + os.path.splitext(filename)[0] + '.png'
                                except IOError:
                                    visual.material.texture = ""
                                    print('failed to open ' + os.path.join(dirname, filename))

        # box
        if hasElement(geometryElement, 'box'):
            visual.geometry.box.x = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[0])
            visual.geometry.box.y = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[1])
            visual.geometry.box.z = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[2])
            link.visual.append(visual)

        # cylinder
        elif hasElement(geometryElement, 'cylinder'):
            visual.geometry.cylinder.radius = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('radius'))
            visual.geometry.cylinder.length = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('length'))
            link.visual.append(visual)

        # sphere
        elif hasElement(geometryElement, 'sphere'):
            visual.geometry.sphere.radius = float(geometryElement.getElementsByTagName('sphere')[0].getAttribute('radius'))
            link.visual.append(visual)

        # create mesh
        elif hasElement(geometryElement, 'mesh'):
            meshfile = geometryElement.getElementsByTagName('mesh')[0].getAttribute('filename')
            # hack for gazebo mesh database
            if meshfile.count('package'):
                idx0 = meshfile.find('package://')
                meshfile = meshfile[idx0 + len('package://'):]
            if geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale'):
                meshScale = geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale').split()
                visual.geometry.scale[0] = float(meshScale[0])
                visual.geometry.scale[1] = float(meshScale[1])
                visual.geometry.scale[2] = float(meshScale[2])

            # Collada file
            if os.path.splitext(meshfile)[1].lower() == '.dae':
                getColladaMesh(meshfile, visual, link)

            # STL file
            elif os.path.splitext(meshfile)[1].lower() == '.stl':
                visual = getSTLMesh(meshfile, visual)
                link.visual.append(visual)


def getCollision(link, node):
    """Parse collision of a link.

    Args:
        link (Link): Link instance.
        node (DOM): XML node.
    """
    for index in range(0, len(node.getElementsByTagName('collision'))):
        collision = Collision()
        collisionElement = node.getElementsByTagName('collision')[index]
        if hasElement(collisionElement, 'origin'):
            if collisionElement.getElementsByTagName('origin')[0].getAttribute('xyz'):
                collision.position = getPosition(collisionElement)
            if collisionElement.getElementsByTagName('origin')[0].getAttribute('rpy'):
                if hasElement(collisionElement.getElementsByTagName('geometry')[0], 'cylinder'):
                    collision.rotation = getRotation(collisionElement, True)
                else:
                    collision.rotation = getRotation(collisionElement)
        elif hasElement(collisionElement.getElementsByTagName('geometry')[0], 'cylinder'):
            collision.rotation = getRotation(collisionElement, True)

        geometryElement = collisionElement.getElementsByTagName('geometry')[0]
        if hasElement(geometryElement, 'box'):
            collision.geometry.box.x = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[0])
            collision.geometry.box.y = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[1])
            collision.geometry.box.z = float(geometryElement.getElementsByTagName('box')[0].getAttribute('size').split()[2])
            link.collision.append(collision)
        elif hasElement(geometryElement, 'cylinder'):
            collision.geometry.cylinder.radius = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('radius'))
            collision.geometry.cylinder.length = float(geometryElement.getElementsByTagName('cylinder')[0].getAttribute('length'))
            link.collision.append(collision)
        elif hasElement(geometryElement, 'sphere'):
            collision.geometry.sphere.radius = float(geometryElement.getElementsByTagName('sphere')[0].getAttribute('radius'))
            link.collision.append(collision)
        elif hasElement(geometryElement, 'mesh'):
            meshfile = geometryElement.getElementsByTagName('mesh')[0].getAttribute('filename')
            if geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale'):
                meshScale = geometryElement.getElementsByTagName('mesh')[0].getAttribute('scale').split()
                collision.geometry.scale[0] = float(meshScale[0])
                collision.geometry.scale[1] = float(meshScale[1])
                collision.geometry.scale[2] = float(meshScale[2])
            # hack for gazebo mesh database
            if meshfile.count('package'):
                idx0 = meshfile.find('package://')
                meshfile = meshfile[idx0 + len('package://'):]
            if os.path.splitext(meshfile)[1] == '.dae':
                collision.geometry.collada = getColladaMesh(meshfile, collision, link)
            elif os.path.splitext(meshfile)[1] == '.stl':
                collision.geometry.stl = getSTLMesh(meshfile, collision)
                link.collision.append(collision)


def getAxis(node):
    """Parse rotation axis of a joint.

    Args:
        node (DOM): XML node.
    """
    axis = [0.0, 0.0, 0.0]
    axisElement = node.getElementsByTagName('axis')[0].getAttribute('xyz').split()
    axis[0] = float(axisElement[0])
    axis[1] = float(axisElement[1])
    axis[2] = float(axisElement[2])
    return axis


def getCalibration(node):
    """Get the URDF calibration tag.

    Args:
        node (DOM): XML node.
    """
    calibration = Calibration()
    calibrationElement = node.getElementsByTagName('calibration')[0]
    if hasElement(calibrationElement, 'rising'):
        calibration.limit = calibrationElement.getAttribute('rising')
        calibration.rising = True
    else:
        calibration.limit = calibrationElement.getAttribute('falling')
        calibration.rising = False
    return calibration


def getDynamics(node):
    """Parse dynamics parameters of a joint.

    Args:
        node (DOM): XML node.
    """
    dynamics = Dynamics()
    dynamicsElement = node.getElementsByTagName('dynamics')[0]
    if dynamicsElement.getAttribute('damping'):
        dynamics.damping = float(dynamicsElement.getAttribute('damping'))
    if dynamicsElement.getAttribute('friction'):
        dynamics.friction = float(dynamicsElement.getAttribute('friction'))
    return dynamics


def getLimit(node):
    """Get limits of a joint.

    Args:
        node (DOM): XML node.
    """
    limit = Limit()
    limitElement = node.getElementsByTagName('limit')[0]
    if limitElement.getAttribute('lower'):
        limit.lower = float(limitElement.getAttribute('lower'))
    if limitElement.getAttribute('upper'):
        limit.upper = float(limitElement.getAttribute('upper'))
    limit.effort = float(limitElement.getAttribute('effort'))
    limit.velocity = float(limitElement.getAttribute('velocity'))
    return limit


def getSafety(node):
    """Get safety controller of a joint.

    Args:
        node (DOM): XML node.
    """
    safety = Safety()
    if node.getElementsByTagName('safety_controller')[0].getAttribute('soft_lower_limit'):
        safety.lower = float(node.getElementsByTagName('safety_controller')[0].getAttribute('soft_lower_limit'))
    if node.getElementsByTagName('safety_controller')[0].getAttribute('soft_upper_limit'):
        safety.upper = float(node.getElementsByTagName('safety_controller')[0].getAttribute('soft_upper_limit'))
    if node.getElementsByTagName('safety_controller')[0].getAttribute('k_position'):
        safety.kPosition = float(node.getElementsByTagName('safety_controller')[0].getAttribute('k_position'))
    safety.kVelocity = float(node.getElementsByTagName('safety_controller')[0].getAttribute('k_velocity'))
    return safety


def getLink(node):
    """Parse a link.

    Args:
        node (DOM): XML node.
    """

    # create link object
    link = Link()
    link.name = node.getAttribute('name')

    # set inertia
    if hasElement(node, 'inertial'):
        link.inertia = getInertia(node)

    # set visual
    if hasElement(node, 'visual'):
        getVisual(link, node)

    # set collision
    if hasElement(node, 'collision'):
        getCollision(link, node)

    # return the created link object
    return link


def getJoint(node):
    """Parse a joint.

    Args:
        node (DOM): XML node.
    """

    # create joint object
    joint = Joint()
    joint.name = node.getAttribute('name')
    joint.type = node.getAttribute('type')
    if hasElement(node, 'origin'):
        if node.getElementsByTagName('origin')[0].getAttribute('xyz'):
            joint.position = getPosition(node)
        if node.getElementsByTagName('origin')[0].getAttribute('rpy'):
            joint.rotation = getRotation(node)
    joint.parent = node.getElementsByTagName('parent')[0].getAttribute('link')
    joint.child = node.getElementsByTagName('child')[0].getAttribute('link')
    if hasElement(node, 'axis'):
        joint.axis = getAxis(node)
    if hasElement(node, 'calibration'):
        joint.calibration = getCalibration(node)
    if hasElement(node, 'dynamics'):
        joint.dynamics = getDynamics(node)
    if hasElement(node, 'limit'):
        joint.limit = getLimit(node)
    if hasElement(node, 'safety_controller'):
        joint.safety = getSafety(node)

    # return joint
    return joint


def isRootLink(link, children):
    """Check if a link is root link."""
    for child in children:
        if link == child:
            return False
    return True


def parseGazeboElement(element, parents, links):
    """Parse a Gazebo element."""
    for plugin in element.getElementsByTagName('plugin'):
        if plugin.hasAttribute('filename') and plugin.getAttribute('filename').startswith('libgazebo_ros_imu'):
            imu = IMU()
            imu.parentLink = parents
            if hasElement(plugin, 'topicName'):
                imu.name = plugin.getElementsByTagName('topicName')[0].firstChild.nodeValue
            if hasElement(plugin, 'gaussianNoise'):
                imu.gaussianNoise = float(plugin.getElementsByTagName('gaussianNoise')[0].firstChild.nodeValue)
            IMU.list.append(imu)
    for sensorElement in element.getElementsByTagName('sensor'):
        sensorElement = element.getElementsByTagName('sensor')[0]
        if sensorElement.getAttribute('type') == 'camera':
            camera = Camera()
            camera.parentLink = parents
            if element.hasAttribute('reference') and element.getAttribute('reference') in links:
                camera.parentLink = element.getAttribute('reference')
            camera.name = sensorElement.getAttribute('name')
            if hasElement(sensorElement, 'camera'):
                cameraElement = sensorElement.getElementsByTagName('camera')[0]
                if hasElement(cameraElement, 'horizontal_fov'):
                    camera.fov = float(cameraElement.getElementsByTagName('horizontal_fov')[0].firstChild.nodeValue)
                if hasElement(cameraElement, 'image'):
                    imageElement = cameraElement.getElementsByTagName('image')[0]
                    if hasElement(imageElement, 'width'):
                        camera.width = int(imageElement.getElementsByTagName('width')[0].firstChild.nodeValue)
                    if hasElement(imageElement, 'height'):
                        camera.height = int(imageElement.getElementsByTagName('height')[0].firstChild.nodeValue)
                    if hasElement(imageElement, 'format') and imageElement.getElementsByTagName('format')[0].firstChild.nodeValue != 'R8G8B8A8':
                        print('Unsupported "%lf" image format, using "R8G8B8A8" instead.' % imageElement.getElementsByTagName('format')[0].firstChild.nodeValue)
            if hasElement(sensorElement, 'noise'):
                noiseElement = sensorElement.getElementsByTagName('noise')[0]
                if hasElement(noiseElement, 'stddev'):
                    camera.noise = float(noiseElement.getElementsByTagName('stddev')[0].firstChild.nodeValue)
            Camera.list.append(camera)
        elif sensorElement.getAttribute('type') == 'ray':
            lidar = Lidar()
            lidar.parentLink = parents
            if element.hasAttribute('reference') and element.getAttribute('reference') in links:
                lidar.parentLink = element.getAttribute('reference')
            lidar.name = sensorElement.getAttribute('name')
            if hasElement(sensorElement, 'ray'):
                rayElement = sensorElement.getElementsByTagName('ray')[0]
                if hasElement(rayElement, 'scan'):
                    scanElement = rayElement.getElementsByTagName('scan')[0]
                    if hasElement(scanElement, 'horizontal'):
                        horizontalElement = scanElement.getElementsByTagName('horizontal')[0]
                        if hasElement(horizontalElement, 'samples'):
                            lidar.horizontalResolution = int(horizontalElement.getElementsByTagName('samples')[0].firstChild.nodeValue)
                        if hasElement(horizontalElement, 'min_angle') and hasElement(horizontalElement, 'max_angle'):
                            minAngle = float(horizontalElement.getElementsByTagName('min_angle')[0].firstChild.nodeValue)
                            maxAngle = float(horizontalElement.getElementsByTagName('max_angle')[0].firstChild.nodeValue)
                            lidar.fov = maxAngle - minAngle
                    if hasElement(scanElement, 'vertical'):
                        horizontalElement = scanElement.getElementsByTagName('horizontal')[0]
                        if hasElement(horizontalElement, 'samples'):
                            lidar.numberOfLayers = int(horizontalElement.getElementsByTagName('samples')[0].firstChild.nodeValue)
                        if hasElement(horizontalElement, 'min_angle') and hasElement(horizontalElement, 'max_angle'):
                            minAngle = float(horizontalElement.getElementsByTagName('min_angle')[0].firstChild.nodeValue)
                            maxAngle = float(horizontalElement.getElementsByTagName('max_angle')[0].firstChild.nodeValue)
                            lidar.verticalFieldOfView = maxAngle - minAngle
                if hasElement(rayElement, 'range'):
                    rangeElement = rayElement.getElementsByTagName('range')[0]
                    if hasElement(rangeElement, 'min'):
                        lidar.minRange = float(rangeElement.getElementsByTagName('min')[0].firstChild.nodeValue)
                    if hasElement(rangeElement, 'max'):
                        lidar.maxRange = float(rangeElement.getElementsByTagName('max')[0].firstChild.nodeValue)
                    if hasElement(rangeElement, 'resolution'):
                        lidar.resolution = float(rangeElement.getElementsByTagName('resolution')[0].firstChild.nodeValue)
                if hasElement(sensorElement, 'noise'):
                    noiseElement = sensorElement.getElementsByTagName('noise')[0]
                    if hasElement(noiseElement, 'stddev'):
                        lidar.noise = float(noiseElement.getElementsByTagName('stddev')[0].firstChild.nodeValue)
                        if lidar.maxRange:
                            lidar.noise /= lidar.maxRange
            Lidar.list.append(lidar)
