import re

iFile = open('./V1_sim_shells_reduced_polygon_count_mit.urdf', 'r')
lines = iFile.readlines()
links = '(Trunk|'\
'RightShoulderExtensor|'\
'RightShoulderAdductor|'\
'RightShoulderRotator|'\
'RightElbowExtensor|'\
'RightWristYoke|'\
'RightPalm|'\
'RightShoulderSupinator|'\
'RightWristExtensor|'\
'RightWrist)'
newFile = ''
newLines = []
addLine = True
RWExt = False
RSExt = False
Trunk = False
visual = False

for line in range(len(lines)):
    matchRobotStart = re.match('.*(<robot name=).*', lines[line])
    matchRobotEnd = re.match('.*</robot>.*', lines[line])
    matchLinkJointStart = re.match('.*(link|joint) name="' + links + '[^/]*$', lines[line])
    matchRWExtensor = re.match('.*joint name="RightWristExtensor.*$', lines[line])
    if matchLinkJointStart or matchRobotEnd:
        addLine = True
        
    if addLine :
        if matchRWExtensor:
            RWExt = True
        if RWExt:
            matchParent = re.match('(.*<parent link=").*("/>.*)', lines[line])
            if matchParent:
                lines[line] = matchParent.group(1) + 'RightElbowExtensor' + matchParent.group(2) + '\n'
        newLines.append(lines[line])
            

    matchLinkJointEnd = re.match('.*(/link|/joint).*', lines[line])
    if matchLinkJointEnd or matchRobotStart:
        addLine = False
        RWExt = False

for line in range(len(newLines)):    
    matchTrunk = re.match('.*link name="Trunk.*$', newLines[line])
    matchVisual = re.match('.*<visual', newLines[line])
    matchRWExtensor = re.match('.*joint name="RightWristExtensor.*$', newLines[line])
    matchRSExtensor = re.match('.*joint name="RightShoulderExtensor.*$', newLines[line])
    if matchTrunk:
        Trunk = True
    if Trunk:
        if matchVisual:
            visual = True
        if visual:
            matchOrigin = re.match('(.*<origin rpy=").*(" xyz=").*("/>.*)', newLines[line])
            if matchOrigin:
                newLines[line] = '{:s}0 -1.57079632679 3.14159265359{:s}0.03157 0.246 -0.29842{:s}\n'.format(
                matchOrigin.group(1), matchOrigin.group(2), matchOrigin.group(3))
                Trunk = False
                visual = False
    if matchRSExtensor:
        RSExt = True
    if RSExt:
        matchOrigin = re.match('(.*<origin rpy=").*(" xyz=").*("/>.*)', newLines[line])
        if matchOrigin:
            newLines[line] = '{:s}1.57079632679 -1.57079632679 3.14159265359{:s}0 0.246 0{:s}\n'.format(
            matchOrigin.group(1), matchOrigin.group(2), matchOrigin.group(3))
            RSExt = False
    if matchRWExtensor:
        RWExt = True
    if RWExt:
        matchOrigin = re.match('(.*<origin rpy=").*(" xyz=").*("/>.*)', newLines[line])
        if matchOrigin:
            newLines[line] = '{:s}3.14159265359 -1.57079632679 0{:s}0.0254 -0.2870 0{:s}\n'.format(
            matchOrigin.group(1), matchOrigin.group(2), matchOrigin.group(3))
            RWExt = False

newFile = ''.join(newLines)

oFile = open('./V1_right_arm_only.urdf', 'w')
oFile.write(newFile)
oFile.close()
iFile.close()