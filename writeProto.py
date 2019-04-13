"""Write proto file."""

import math
import math_utils
import numpy as np


class RGB(object):
    """RGB color object."""

    def __init__(self, red=0.5, green=0.5, blue=0.5):
        """Initialization."""
        self.red = red
        self.green = green
        self.blue = blue


# ref: https://marcodiiga.github.io/rgba-to-rgb-conversion
def RGBA2RGB(RGBA_color, RGB_background=RGB()):
    """Convert RGBA to RGB expression."""
    alpha = RGBA_color.alpha

    new_color = RGB()
    new_color.red = (1 - alpha) * RGB_background.red + alpha * RGBA_color.red
    new_color.green = (1 - alpha) * RGB_background.green + alpha * RGBA_color.green
    new_color.blue = (1 - alpha) * RGB_background.blue + alpha * RGBA_color.blue

    return new_color


def header(proto, src_file, robot_name):
    """Specify VRML file header.

    Args:
        proto (file): proto file.
        src_file (str): URDF source file.
        robot_name (str): name of the robot.
    """
    proto.write('#VRML_SIM R2019a utf8\n')
    proto.write('# license: Apache License 2.0\n')
    proto.write('# license url: http://www.apache.org/licenses/LICENSE-2.0\n')
    proto.write('# This is a proto file for Webots for the ' + robot_name + '\n')
    proto.write('# Extracted from: ' + src_file + '\n\n')


def declaration(proto, robot_name):
    """Prototype declaration.

    Args:
        proto (file): proto file to write information in.
        robot_name (str): name of the robot.
    """
    proto.write('PROTO ' + robot_name + ' [\n')
    proto.write('  field  SFVec3f     translation  0 0 0\n')
    proto.write('  field  SFRotation  rotation     0 1 0 0\n')
    proto.write('  field  SFString    controller   "void"\n')
    proto.write(']\n')
    proto.write('{\n')


def URDFLink(proto, link, level, parents, children, links, joints, sensors,
             jointPosition=[0.0, 0.0, 0.0], jointRotation=[1.0, 0.0, 0.0, 0.0],
             boxCollision=False, normal=False, dummy=False, robot=False, endpoint=False):
    """
    Write a link (and joints) iteratively.

    Args:
        proto (file): proto file to write in.
        link (Link): current link that is being considered.
        level (int): level in the tree.
        parents (list of str): for each joint in :attr:`joints`, its parent joint's name.
        children (list of str): for each joint in :attr:`joints`, its child joint's name.
        links (list of Link): list of link objects.
        joints (list of Joint): list of joint objects.
        sensors (list of IMU/Camera/Lidar): list of sensor objects.
        jointPosition (list of 3 float): joint position
        jointRotation (list of 4 float): joint orientation expressed as a quaternion (w,x,y,z)
        boxCollision (bool): If True, the bounding objects are approximated using boxes.
        normal (bool): If True, the normals are exported if present in the URDF definition.
        dummy (bool): If True, we have a dummy link (which are often used in URDF files to create a frame).
        robot (bool): If True, we have the root link, else we have other links.
        endpoint (bool): If True, it is an end point.
    """
    indent = '  '
    haveChild = False

    if robot:
        proto.write(level * indent + 'Robot {\n')
        proto.write((level + 1) * indent + 'translation IS translation\n')
        proto.write((level + 1) * indent + 'rotation IS rotation\n')
        proto.write((level + 1) * indent + 'controller IS controller\n')
    else:
        proto.write((' ' if endpoint else level * indent) + 'Solid {\n')
        proto.write((level + 1) * indent + 'translation %lf %lf %lf\n' % (jointPosition[0], jointPosition[1], jointPosition[2]))
        proto.write((level + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (jointRotation[0], jointRotation[1], jointRotation[2], jointRotation[3]))

    if dummy:  # case when link not defined but referenced (e.g. Atlas robot)
        pass
    else:
        # for each joint, write the joint element in the proto file
        for joint in joints:
            if joint.parent == link.name:
                if not haveChild:
                    haveChild = True
                    proto.write((level + 1) * indent + 'children [\n')
                URDFJoint(proto, joint, level + 2, parents, children,
                          links, joints, sensors, boxCollision, normal)

        # write visual element in the proto file
        if link.visual:
            if not haveChild:
                haveChild = True
                proto.write((level + 1) * indent + 'children [\n')
            URDFShape(proto, link=link, level=level + 2, normal=normal)

        # for each sensor
        for sensor in sensors:
            if sensor.parentLink == link.name:
                if not haveChild:
                    haveChild = True
                    proto.write((level + 1) * indent + 'children [\n')
                sensor.export(proto, level + 2)

        if haveChild:
            proto.write((level + 1) * indent + ']\n')

        proto.write((level + 1) * indent + 'name "' + link.name + '"\n')

        # for each collision shape
        if link.collision:
            URDFBoundingObject(proto, link=link, level=level + 1, boxCollision=boxCollision)

        # write physics info (mass, density, inertia)
        proto.write((level + 1) * indent + 'physics Physics {\n')
        proto.write((level + 2) * indent + 'density -1\n')
        proto.write((level + 2) * indent + 'mass %lf\n' % link.inertia.mass)
        if link.inertia.ixx > 0.0 and link.inertia.iyy > 0.0 and link.inertia.izz > 0.0:
            proto.write((level + 2) * indent + 'centerOfMass [ %lf %lf %lf ]\n' % (link.inertia.position[0], link.inertia.position[1], link.inertia.position[2]))
        proto.write((level + 1) * indent + '}\n')

        if link.inertia.rotation[-1] != 0.0:  # this should not happend
            print('Warning: inertia of %s has a non-zero rotation [axis-angle] = "%lf %lf %lf %lf" but it will not be '
                  'imported in proto!' % (link.name, link.inertia.rotation[0], link.inertia.rotation[1], link.inertia.rotation[2], link.inertia.rotation[3]))

    proto.write(level * indent + '}\n')


def URDFBoundingObject(proto, link, level, boxCollision):
    """Write a bounding object (collision).

    Args:
        proto (file): proto file to write the information in.
        link (Link): link object.
        level (int): level in the tree.
        boxCollision (bool): If True, the bounding objects are approximated using boxes.
    """
    indent = '  '
    boundingLevel = level
    proto.write(level * indent + 'boundingObject ')
    hasGroup = len(link.collision) > 1
    if hasGroup:
        proto.write('Group {\n')
        proto.write((level + 1) * indent + 'children [\n')
        boundingLevel = level + 2

    for boundingObject in link.collision:
        initialIndent = boundingLevel * indent if hasGroup else ''
        if boundingObject.position != [0.0, 0.0, 0.0] or boundingObject.rotation[3] != 0.0:
            proto.write(initialIndent + 'Transform {\n')
            proto.write((boundingLevel + 1) * indent + 'translation %lf %lf %lf\n' % (boundingObject.position[0], boundingObject.position[1], boundingObject.position[2]))
            proto.write((boundingLevel + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (boundingObject.rotation[0], boundingObject.rotation[1], boundingObject.rotation[2], boundingObject.rotation[3]))
            proto.write((boundingLevel + 1) * indent + 'children [\n')
            boundingLevel = boundingLevel + 2
            hasGroup = True
            initialIndent = boundingLevel * indent

        if boundingObject.geometry.box.x != 0:
            proto.write(initialIndent + 'Box {\n')
            proto.write((boundingLevel + 1) * indent + ' size %lf %lf %lf\n' % (boundingObject.geometry.box.x, boundingObject.geometry.box.y, boundingObject.geometry.box.z))
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.cylinder.radius != 0 and boundingObject.geometry.cylinder.length != 0:
            proto.write(initialIndent + 'Cylinder {\n')
            proto.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.cylinder.radius) + '\n')
            proto.write((boundingLevel + 1) * indent + 'height ' + str(boundingObject.geometry.cylinder.length) + '\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.sphere.radius != 0:
            proto.write(initialIndent + 'Sphere {\n')
            proto.write((boundingLevel + 1) * indent + 'radius ' + str(boundingObject.geometry.sphere.radius) + '\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord and boxCollision:
            aabb = {
                'minimum': {'x': float('inf'),
                            'y': float('inf'),
                            'z': float('inf')},
                'maximum': {'x': float('-inf'),
                            'y': float('-inf'),
                            'z': float('-inf')}
            }
            for value in boundingObject.geometry.trimesh.coord:
                x = value[0] * boundingObject.geometry.scale[0]
                y = value[1] * boundingObject.geometry.scale[1]
                z = value[2] * boundingObject.geometry.scale[2]
                aabb['minimum']['x'] = min(aabb['minimum']['x'], x)
                aabb['maximum']['x'] = max(aabb['maximum']['x'], x)
                aabb['minimum']['y'] = min(aabb['minimum']['y'], y)
                aabb['maximum']['y'] = max(aabb['maximum']['y'], y)
                aabb['minimum']['z'] = min(aabb['minimum']['z'], z)
                aabb['maximum']['z'] = max(aabb['maximum']['z'], z)

            proto.write(initialIndent + 'Transform {\n')
            proto.write((boundingLevel + 1) * indent + 'translation %f %f %f\n' % (
                        0.5 * (aabb['maximum']['x'] + aabb['minimum']['x']),
                        0.5 * (aabb['maximum']['y'] + aabb['minimum']['y']),
                        0.5 * (aabb['maximum']['z'] + aabb['minimum']['z']),))
            proto.write((boundingLevel + 1) * indent + 'children [\n')
            proto.write((boundingLevel + 2) * indent + 'Box {\n')
            proto.write((boundingLevel + 3) * indent + 'size %f %f %f\n' % (
                        aabb['maximum']['x'] - aabb['minimum']['x'],
                        aabb['maximum']['y'] - aabb['minimum']['y'],
                        aabb['maximum']['z'] - aabb['minimum']['z'],))
            proto.write((boundingLevel + 2) * indent + '}\n')
            proto.write((boundingLevel + 1) * indent + ']\n')
            proto.write(boundingLevel * indent + '}\n')

        elif boundingObject.geometry.trimesh.coord:
            proto.write(initialIndent + 'IndexedFaceSet {\n')

            proto.write((boundingLevel + 1) * indent + 'coord Coordinate {\n')
            proto.write((boundingLevel + 2) * indent + 'point [\n' + (boundingLevel + 3) * indent)
            for value in boundingObject.geometry.trimesh.coord:
                proto.write('%lf %lf %lf, ' % (value[0] * boundingObject.geometry.scale[0], value[1] * boundingObject.geometry.scale[1], value[2] * boundingObject.geometry.scale[2]))
            proto.write('\n' + (boundingLevel + 2) * indent + ']\n')
            proto.write((boundingLevel + 1) * indent + '}\n')

            proto.write((boundingLevel + 1) * indent + 'coordIndex [\n' + (boundingLevel + 2) * indent)
            if isinstance(boundingObject.geometry.trimesh.coordIndex[0], np.ndarray) or type(boundingObject.geometry.trimesh.coordIndex[0]) == list:
                for value in boundingObject.geometry.trimesh.coordIndex:
                    if len(value) == 3:
                        proto.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
            elif isinstance(boundingObject.geometry.trimesh.coordIndex[0], np.int32):
                for i in range(len(boundingObject.geometry.trimesh.coordIndex) / 3):
                    proto.write('%d %d %d -1 ' % (boundingObject.geometry.trimesh.coordIndex[3 * i + 0], boundingObject.geometry.trimesh.coordIndex[3 * i + 1], boundingObject.geometry.trimesh.coordIndex[3 * i + 2]))
            else:
                print('Unsupported "%s" coordinate type' % type(boundingObject.geometry.trimesh.coordIndex[0]))
            proto.write('\n' + (boundingLevel + 1) * indent + ']\n')
            proto.write(boundingLevel * indent + '}\n')

        else:
            proto.write(initialIndent + 'Box{\n')
            proto.write((boundingLevel + 1) * indent + ' size 0.01 0.01 0.01\n')
            proto.write(boundingLevel * indent + '}\n')

        if boundingLevel == level + 4:
            proto.write((level + 3) * indent + ']\n')
            proto.write((level + 2) * indent + '}\n')
            boundingLevel = level + 2
    if boundingLevel == level + 2:
        proto.write((level + 1) * indent + ']\n')
        proto.write(level * indent + '}\n')


def URDFShape(proto, link, level, normal=False):
    """Write a Shape.

    Args:
        proto (file): proto file to write the information in.
        link (Link): link object.
        level (int): level in the tree.
        normal (bool):  If True, the normals are exported if present in the URDF definition.
    """
    indent = '  '
    shapeLevel = level
    transform = False

    for visualNode in link.visual:
        if visualNode.position != [0.0, 0.0, 0.0] or visualNode.rotation[3] != 0:
            proto.write(shapeLevel * indent + 'Transform {\n')
            proto.write((shapeLevel + 1) * indent + 'translation %lf %lf %lf\n' % (visualNode.position[0], visualNode.position[1], visualNode.position[2]))
            proto.write((shapeLevel + 1) * indent + 'rotation %lf %lf %lf %lf\n' % (visualNode.rotation[0], visualNode.rotation[1], visualNode.rotation[2], visualNode.rotation[3]))
            proto.write((shapeLevel + 1) * indent + 'children [\n')
            shapeLevel += 2
            transform = True

        proto.write(shapeLevel * indent + 'Shape {\n')
        proto.write((shapeLevel + 1) * indent + 'appearance PBRAppearance {\n')
        ambientColor = RGBA2RGB(visualNode.material.ambient)
        diffuseColor = RGBA2RGB(visualNode.material.diffuse, RGB_background=ambientColor)
        emissiveColor = RGBA2RGB(visualNode.material.emission, RGB_background=ambientColor)
        roughness = 1.0 - visualNode.material.specular.alpha * (visualNode.material.specular.red + visualNode.material.specular.green + visualNode.material.specular.blue) / 3.0
        if visualNode.material.shininess:
            roughness *= (1.0 - 0.5 * visualNode.material.shininess)
        proto.write((shapeLevel + 2) * indent + 'baseColor %lf %lf %lf\n' % (diffuseColor.red, diffuseColor.green, diffuseColor.blue))
        proto.write((shapeLevel + 2) * indent + 'transparency %lf\n' % (1.0 - visualNode.material.diffuse.alpha))
        proto.write((shapeLevel + 2) * indent + 'roughness %lf\n' % roughness)
        proto.write((shapeLevel + 2) * indent + 'metalness 0\n')
        proto.write((shapeLevel + 2) * indent + 'emissiveColor %lf %lf %lf\n' % (emissiveColor.red, emissiveColor.green, emissiveColor.blue))
        if visualNode.material.texture != "":
            proto.write((shapeLevel + 2) * indent + 'baseColorMap ImageTexture {\n')
            proto.write((shapeLevel + 3) * indent + 'url [ "' + visualNode.material.texture + '" ]\n')
            proto.write((shapeLevel + 2) * indent + '}\n')
        proto.write((shapeLevel + 1) * indent + '}\n')

        if visualNode.geometry.box.x != 0:
            proto.write((shapeLevel + 1) * indent + 'geometry Box {\n')
            proto.write((shapeLevel + 2) * indent + ' size ' +
                        str(visualNode.geometry.box.x) + ' ' +
                        str(visualNode.geometry.box.y) + ' ' +
                        str(visualNode.geometry.box.z) + '\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        elif visualNode.geometry.cylinder.radius != 0:
            proto.write((shapeLevel + 1) * indent + 'geometry Cylinder {\n')
            proto.write((shapeLevel + 2) * indent + 'radius ' + str(visualNode.geometry.cylinder.radius) + '\n')
            proto.write((shapeLevel + 2) * indent + 'height ' + str(visualNode.geometry.cylinder.length) + '\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        elif visualNode.geometry.sphere.radius != 0:
            proto.write((shapeLevel + 1) * indent + 'geometry Sphere {\n')
            proto.write((shapeLevel + 2) * indent + 'radius ' + str(visualNode.geometry.sphere.radius) + '\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        elif visualNode.geometry.trimesh.coord:
            proto.write((shapeLevel + 1) * indent + 'geometry IndexedFaceSet {\n')
            proto.write((shapeLevel + 2) * indent + 'coord Coordinate {\n')
            proto.write((shapeLevel + 3) * indent + 'point [\n' + (shapeLevel + 4) * indent)
            for value in visualNode.geometry.trimesh.coord:
                proto.write('%lf %lf %lf, ' % (value[0] * visualNode.geometry.scale[0], value[1] * visualNode.geometry.scale[1], value[2] * visualNode.geometry.scale[2]))
            proto.write('\n' + (shapeLevel + 3) * indent + ']\n')
            proto.write((shapeLevel + 2) * indent + '}\n')

            proto.write((shapeLevel + 2) * indent + 'coordIndex [\n' + (shapeLevel + 3) * indent)
            if isinstance(visualNode.geometry.trimesh.coordIndex[0], np.ndarray) or type(visualNode.geometry.trimesh.coordIndex[0]) == list:
                for value in visualNode.geometry.trimesh.coordIndex:
                    if len(value) == 3:
                        proto.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
            elif isinstance(visualNode.geometry.trimesh.coordIndex[0], np.int32):
                for i in range(len(visualNode.geometry.trimesh.coordIndex) / 3):
                    proto.write('%d %d %d -1 ' % (visualNode.geometry.trimesh.coordIndex[3 * i + 0], visualNode.geometry.trimesh.coordIndex[3 * i + 1], visualNode.geometry.trimesh.coordIndex[3 * i + 2]))
            else:
                print('Unsupported "%s" coordinate type' % type(visualNode.geometry.trimesh.coordIndex[0]))
            proto.write('\n' + (shapeLevel + 2) * indent + ']\n')

            if normal and visualNode.geometry.trimesh.normal and visualNode.geometry.trimesh.normalIndex:
                proto.write((shapeLevel + 2) * indent + 'normal Normal {\n')
                proto.write((shapeLevel + 3) * indent + 'vector [\n' + (shapeLevel + 4) * indent)
                for value in visualNode.geometry.trimesh.normal:
                    proto.write('%lf %lf %lf, ' % (value[0], value[1], value[2]))
                proto.write('\n' + (shapeLevel + 3) * indent + ']\n')
                proto.write((shapeLevel + 2) * indent + '}\n')

                proto.write((shapeLevel + 2) * indent + 'normalIndex [\n' + (shapeLevel + 3) * indent)
                if isinstance(visualNode.geometry.trimesh.normalIndex[0], np.ndarray) or type(visualNode.geometry.trimesh.normalIndex[0]) == list:
                    for value in visualNode.geometry.trimesh.normalIndex != 0:
                        if len(value) == 3:
                            proto.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
                elif isinstance(visualNode.geometry.trimesh.normalIndex[0], np.int32):
                    for i in range(len(visualNode.geometry.trimesh.normalIndex) / 3):
                        proto.write('%d %d %d -1 ' % (visualNode.geometry.trimesh.normalIndex[3 * i + 0], visualNode.geometry.trimesh.normalIndex[3 * i + 1], visualNode.geometry.trimesh.normalIndex[3 * i + 2]))
                else:
                    print('Unsupported "%s" normal type' % type(visualNode.geometry.trimesh.normalIndex[0]))
                proto.write('\n' + (shapeLevel + 2) * indent + ']\n')

            if visualNode.geometry.trimesh.texCoord:
                proto.write((shapeLevel + 2) * indent + 'texCoord TextureCoordinate {\n')
                proto.write((shapeLevel + 3) * indent + 'point [\n' + (shapeLevel + 4) * indent)
                for value in visualNode.geometry.trimesh.texCoord:
                    proto.write('%lf %lf, ' % (value[0], value[1]))
                proto.write('\n' + (shapeLevel + 3) * indent + ']\n')
                proto.write((shapeLevel + 2) * indent + '}\n')

                proto.write((shapeLevel + 2) * indent + 'texCoordIndex [\n' + (shapeLevel + 3) * indent)
                if isinstance(visualNode.geometry.trimesh.texCoordIndex[0], np.ndarray) or type(visualNode.geometry.trimesh.texCoordIndex[0]) == list:
                    for value in visualNode.geometry.trimesh.texCoordIndex:
                        if len(value) == 3:
                            proto.write('%d %d %d -1 ' % (value[0], value[1], value[2]))
                elif isinstance(visualNode.geometry.trimesh.texCoordIndex[0], np.int32):
                    for i in range(len(visualNode.geometry.trimesh.texCoordIndex) / 3):
                        proto.write('%d %d %d -1 ' % (visualNode.geometry.trimesh.texCoordIndex[3 * i + 0], visualNode.geometry.trimesh.texCoordIndex[3 * i + 1], visualNode.geometry.trimesh.texCoordIndex[3 * i + 2]))
                else:
                    print('Unsupported "%s" coordinate type' % type(visualNode.geometry.trimesh.texCoordIndex[0]))
                proto.write('\n' + (shapeLevel + 2) * indent + ']\n')

            proto.write((shapeLevel + 2) * indent + 'creaseAngle 1\n')
            proto.write((shapeLevel + 1) * indent + '}\n')

        proto.write(shapeLevel * indent + '}\n')
        if transform:
            proto.write((shapeLevel - 1) * indent + ']\n')
            proto.write((shapeLevel - 2) * indent + '}\n')
            shapeLevel -= 2


def URDFJoint(proto, joint, level, parents, children, links, joints, sensors, boxCollision, normal):
    """Write a Joint iteratively.

    Args:
        proto (file): proto file to write the information in.
        joint (Joint): joint object.
        level (int): level in the tree.
        parents (list of str): for each joint in :attr:`joints`, its parent joint's name.
        children (list of str): for each joint in :attr:`joints`, its child joint's name.
        links (list of Link): list of link objects.
        joints (list of Joint): list of joint objects.
        sensors (list of IMU/Camera/Lidar): list of sensor objects.
        boxCollision (bool): If True, the bounding objects are approximated using boxes.
        normal (bool): If True, the normals are exported if present in the URDF definition.
    """
    indent = '  '
    if not joint.axis:
        joint.axis = [1, 0, 0]
    axis = joint.axis
    endpointRotation = joint.rotation
    endpointPosition = joint.position

    if joint.rotation[3] != 0.0 and axis:
        axis = math_utils.rotateVector(axis, joint.rotation)

    # revolute joint
    if joint.type == 'revolute' or joint.type == 'continuous':
        proto.write(level * indent + 'HingeJoint {\n')
        proto.write((level + 1) * indent + 'jointParameters HingeJointParameters {\n')
        if joint.limit.lower > 0.0:
            # if 0 is not in the range, set the position to be the middle of the range
            position = joint.limit.lower
            if joint.limit.upper >= joint.limit.lower:
                position = (joint.limit.upper - joint.limit.lower) / 2.0 + joint.limit.lower
            proto.write((level + 2) * indent + 'position %lf \n' % position)
            mat1 = math_utils.matrixFromRotation(endpointRotation)
            mat2 = math_utils.matrixFromRotation([axis[0], axis[1], axis[2], position])
            mat3 = math_utils.multiplyMatrix(mat2, mat1)
            endpointRotation = math_utils.rotationFromMatrix(mat3)
        proto.write((level + 2) * indent + 'axis %lf %lf %lf\n' % (axis[0], axis[1], axis[2]))
        proto.write((level + 2) * indent + 'anchor %lf %lf %lf\n' % (joint.position[0], joint.position[1], joint.position[2]))
        proto.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        proto.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        proto.write((level + 1) * indent + '}\n')
        proto.write((level + 1) * indent + 'device [\n')
        proto.write((level + 2) * indent + 'RotationalMotor {\n')

    # prismatic joint
    elif joint.type == 'prismatic':
        proto.write(level * indent + 'SliderJoint {\n')
        proto.write((level + 1) * indent + 'jointParameters JointParameters {\n')
        if joint.limit.lower > 0.0:
            # if 0 is not in the range, set the position to be the middle of the range
            position = joint.limit.lower
            if joint.limit.upper >= joint.limit.lower:
                position = (joint.limit.upper - joint.limit.lower) / 2.0 + joint.limit.lower
            proto.write((level + 2) * indent + 'position %lf \n' % position)
            length = math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2])
            if length > 0:
                endpointPosition[0] += axis[0] / length * position
                endpointPosition[0] += axis[1] / length * position
                endpointPosition[0] += axis[2] / length * position
        proto.write((level + 2) * indent + 'axis %lf %lf %lf\n' % (axis[0], axis[1], axis[2]))
        proto.write((level + 2) * indent + 'dampingConstant ' + str(joint.dynamics.damping) + '\n')
        proto.write((level + 2) * indent + 'staticFriction ' + str(joint.dynamics.friction) + '\n')
        proto.write((level + 1) * indent + '}\n')
        proto.write((level + 1) * indent + 'device [\n')
        proto.write((level + 2) * indent + 'LinearMotor {\n')

    # fixed joint
    elif joint.type == 'fixed':
        for childLink in links:
            if childLink.name == joint.child:
                URDFLink(proto, childLink, level, parents, children,
                         links, joints, sensors, joint.position, joint.rotation,
                         boxCollision, normal)
        return

    # floating or planar joint
    elif joint.type == 'floating' or joint.type == 'planar':
        print(joint.type + ' is not a supported joint type in Webots')
        return

    # write joint limit
    proto.write((level + 3) * indent + 'name "' + joint.name + '"\n')
    if joint.limit.velocity != 0.0:
        proto.write((level + 3) * indent + 'maxVelocity ' + str(joint.limit.velocity) + '\n')
    if joint.limit.lower != 0.0:
        proto.write((level + 3) * indent + 'minPosition ' + str(joint.limit.lower) + '\n')
    if joint.limit.upper != 0.0:
        proto.write((level + 3) * indent + 'maxPosition ' + str(joint.limit.upper) + '\n')
    if joint.limit.effort != 0.0:
        if joint.type == 'prismatic':
            proto.write((level + 3) * indent + 'maxForce ' + str(joint.limit.effort) + '\n')
        else:
            proto.write((level + 3) * indent + 'maxTorque ' + str(joint.limit.effort) + '\n')
    proto.write((level + 2) * indent + '}\n')
    proto.write((level + 2) * indent + 'PositionSensor {\n')
    proto.write((level + 3) * indent + 'name "' + joint.name + '_sensor"\n')
    proto.write((level + 2) * indent + '}\n')
    proto.write((level + 1) * indent + ']\n')

    proto.write((level + 1) * indent + 'endPoint')
    found_link = False
    for childLink in links:
        if childLink.name == joint.child:
            URDFLink(proto, childLink, level + 1, parents, children,
                     links, joints, sensors, endpointPosition, endpointRotation,
                     boxCollision, normal, endpoint=True)
            assert(not found_link)
            found_link = True
    # case that non-existing link cited, set dummy flag
    if not found_link and joint.child:
        URDFLink(proto, joint.child, level + 1, parents, children,
                 links, joints, sensors, endpointPosition, endpointRotation,
                 boxCollision, normal, dummy=True)
        print('warning: link ' + joint.child + ' is dummy!')
    proto.write(level * indent + '}\n')
