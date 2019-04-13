#!/usr/bin/env python
"""URDF files to Webots PROTO converter."""


import os
import errno
import re
import sys
import optparse
import parserURDF
import writeProto
from xml.dom import minidom


def convertLUtoUN(s):
    """Capitalize a string and remove any underscores.

    If 'hello_world' is given, it will return 'HelloWorld'.
    """
    return ''.join([word.capitalize() for word in s.split('_')])


def mkdir_safe(directory):
    """Create a dir safely."""
    try:
        os.makedirs(directory)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
        else:
            print('Directory "' + directory + '" already exists!')


# define the parsing arguments
optParser = optparse.OptionParser(usage='usage: %prog --input=my_robot.urdf [options]')
optParser.add_option('--input', dest='inFile', default='', help='Specifies the urdf file to convert.')
optParser.add_option('--output', dest='outFile', default='', help='Specifies the name of the resulting PROTO file.')
optParser.add_option('--normal', dest='normal', action='store_true', default=False,
                     help='If set, the normals are exported if present in the URDF definition.')
optParser.add_option('--box-collision', dest='boxCollision', action='store_true', default=False,
                     help='If set, the bounding objects are approximated using boxes.')
options, args = optParser.parse_args()

# check that the input file is provided and exists
if not options.inFile:
    sys.exit('--input argument missing.')
if not os.path.exists(options.inFile):
    sys.exit('Input file "%s" does not exists.' % options.inFile)

# open urdf file
with open(options.inFile, 'r') as file:
    # read the urdf file content
    content = file.read()

    # check in the urdfs if there are relative paths to the textures, meshes, and others
    # and replace them by their absolute paths
    packages = re.findall('"package://(.*)"', content)
    if packages:
        packageName = packages[0].split('/')[0]
        directory = os.path.dirname(options.inFile)
        while packageName != os.path.split(directory)[1] and os.path.split(directory)[1]:
            directory = os.path.dirname(directory)
        if os.path.split(directory)[1]:
            packagePath = os.path.split(directory)[0]
            content = content.replace('package:/', packagePath)
        else:
            sys.stderr.write('Can\'t determine package root path.\n')

    # Parse the URDF xml file
    domFile = minidom.parseString(content)

    # go through each XML node
    for child in domFile.childNodes:

        # if robot node
        if child.localName == 'robot':
            robotName = convertLUtoUN(parserURDF.getRobotName(child))  # capitalize
            outputFile = options.outFile if options.outFile else robotName + '.proto'

            parserURDF.robotName = robotName  # pass robotName
            mkdir_safe(outputFile.replace('.proto', '') + '_textures')  # make a dir called 'x_textures'

            # open output file
            robot = child
            protoFile = open(outputFile, 'w')

            # write header in proto file
            writeProto.header(protoFile, options.inFile, robotName)

            # go through each elements in the robot xml node and check if they are links, joints or materials
            links, joints = [], []
            for child in robot.childNodes:
                if child.localName == 'link':
                    links.append(child)
                elif child.localName == 'joint':
                    joints.append(child)
                elif child.localName == 'material':
                    material = parserURDF.Material()
                    material.parseFromMaterialNode(child)

            # create Joint and Link instances
            rootLink = parserURDF.Link()
            joints = [parserURDF.getJoint(joint) for joint in joints]
            parents = [joint.parent.encode('ascii') for joint in joints]
            children = [joint.child.encode('ascii') for joint in joints]
            parents.sort()
            children.sort()
            links = [parserURDF.getLink(link) for link in links]

            # check if root link has only one joint which type is fixed. If so, it should not be part of the model
            # (link between robot and static environment)
            # so if you have: world --fixed joint--> frame1 --fixed joint--> base
            # the root link will be set to be the base
            for link in links:
                if parserURDF.isRootLink(link.name, children):
                    rootLink = link

                    while True:
                        found = False  # To avoid endless loop
                        # get joints that are the child of the root link
                        directJoint = [joint for joint in joints if joint.parent == rootLink.name]

                        if len(directJoint) == 1 and directJoint[0].type == 'fixed':
                            for childLink in links:
                                if childLink.name == directJoint[0].child:
                                    rootLink = childLink  # the child link is now the root link
                                    found = True
                                    break
                        else:
                            break
                        if not found:
                            break
                    print('Root link: ' + rootLink.name)
                    break

            # check gazebo elements in the XML (for sensors such as IMU, Lidar, Camera, etc)
            for child in robot.childNodes:
                if child.localName == 'gazebo':
                    parserURDF.parseGazeboElement(child, rootLink.name, links)

            # get the sensors
            sensors = parserURDF.IMU.list + parserURDF.Camera.list + parserURDF.Lidar.list
            print('There are %d links, %d joints and %d sensors' % (len(links), len(joints), len(sensors)))

            # write proto file

            # write declaration
            writeProto.declaration(protoFile, robotName)

            # write iteratively each link in the proto file
            writeProto.URDFLink(protoFile, link=rootLink, level=1, parents=parents, children=children, links=links,
                                joints=joints, sensors=sensors, boxCollision=options.boxCollision,
                                normal=options.normal, robot=True)

            # write end of proto file
            protoFile.write('}\n')
            protoFile.close()
            exit()

print('Could not read file')
