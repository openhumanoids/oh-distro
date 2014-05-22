import robotiqhand
import time

class commandLcmToModbusConverter(object):

    def __init__ (self):

        #Byte 0
        self.rACT = 0  # Activate: 1 bit, 0=reset, 1=activate
        self.rMOD = 0  # Mode: 2 bits, 00=basic, 10=pinch, 01=wide, 11=scissor
        self.rGTO = 0  # Goto: 1 bit, 0=stop, 1=goto requested position
        self.rATR = 0  # Release: 1 bit, 0=normal release, 1=automatic release
        self.rRS0 = 0  # Reserved: bits 5-7

        #Byte 1
        self.rGLV = 0  # Glove mode: 1 bit, 0=normal, 1=glove mode
        self.rRS1 = 0  # Reserved: 1 bit
        self.rICF = 0  # Finger Individual mode: 1 bit, 0=normal, 1=individual
        self.rICS = 0  # Scissor Individual mode: 1 bit, 0=normal, 1=individual
        self.rRS2 = 0  # Reserved bits 4-7

        #Byte 2
        self.rRS3 = 0  # Reserved bits 0-7

        #Bytes 3-5
        self.rPRA = 0  # Position of finger A (or all in simple mode)
        self.rSPA = 0  # Speed of finger A (or all in simple mode)
        self.rFRA = 0  # Force of finger A (or all in simple mode)

        #Bytes 6-8
        self.rPRB = 0  # Position of finger B
        self.rSPB = 0  # Speed of finger B
        self.rFRB = 0  # Force of finger B

        #Bytes 9-11
        self.rPRC = 0  # Position of finger C
        self.rSPC = 0  # Speed of finger C
        self.rFRC = 0  # Force of finger C

        #Bytes 12-14
        self.rPRS = 0  # Position of scissor fingers
        self.rSPS = 0  # Speed of scissor fingers
        self.rFRS = 0  # Force of scissor fingers

        #State variables
        self.mode = 0
        self.position = 0
        self.speed = 128
        self.force = 128

        #State variables for individual control mode
        self.ifc = 0
        self.positionA = 0
        self.positionB = 0
        self.positionC = 0
        self.isc = 0
        self.positionS = 0


    def verifyCommand(self):

        fullBytes = ['rPRA', 'rSPA', 'rFRA',
                     'rPRB', 'rSPB', 'rFRB',
                     'rPRC', 'rSPC', 'rFRC',
                     'rPRS', 'rSPS', 'rFRS']
        oneBit = ['rACT', 'rGTO', 'rATR',
                  'rGLV', 'rICF', 'rICS']
        twoBits = ['rMOD']

        for word in fullBytes:
            self.__setattr__(word, min(255, self.__getattribute__(word)))
            self.__setattr__(word, max(0, self.__getattribute__(word)))

        for word in oneBit:
            self.__setattr__(word, min(1, self.__getattribute__(word)))
            self.__setattr__(word, max(0, self.__getattribute__(word)))

        for word in twoBits:
            self.__setattr__(word, min(3, self.__getattribute__(word)))
            self.__setattr__(word, max(0, self.__getattribute__(word)))

    def parseLcm(self, lcmCommand):

        if lcmCommand.mode != -1 and self.mode != lcmCommand.mode:
            self.mode = lcmCommand.mode
            self.position = 0

        self.rMOD = self.mode

        self.rACT = lcmCommand.activate

        self.rATR = lcmCommand.emergency_release

        if lcmCommand.do_move == 1:
            self.position = lcmCommand.position
            self.force = lcmCommand.force
            self.speed = lcmCommand.velocity

        self.rGTO = lcmCommand.do_move

        self.rICF = lcmCommand.ifc
        self.rICS = lcmCommand.isc

        if lcmCommand.ifc == 0:
            self.rPRA = self.position
            self.rPRB = self.position
            self.rPRC = self.position

            self.rSPA = self.speed
            self.rSPB = self.speed
            self.rSPC = self.speed

            self.rFRA = self.force
            self.rFRB = self.force
            self.rFRC = self.force

            self.rPRS = self.position
            self.rSPS = self.speed
            self.rFRS = self.force

        elif lcmCommand.ifc == 1:
            self.rPRA = lcmCommand.positionA
            self.rPRB = lcmCommand.positionB
            self.rPRC = lcmCommand.positionC

            self.rSPA = self.speed
            self.rSPB = self.speed
            self.rSPC = self.speed
            self.rSPS = self.speed

            self.rFRA = self.force
            self.rFRB = self.force
            self.rFRC = self.force
            self.rFRS = self.force

        if lcmCommand.isc == 1:
            self.rPRS = lcmCommand.positionS
            self.rSPS = self.speed
            self.rFRS = self.force

        #Data populated, now verify
        self.verifyCommand()

    def getModbusString(self):
        #Initiate command as an empty list
        message = []

        #Build the command with each output variable
        message.append(self.rACT + (self.rMOD << 1) + (self.rGTO << 3) + (self.rATR << 4))
        message.append(self.rGLV + (self.rICF << 2) + (self.rICS << 3))
        message.append(0)
        message.append(self.rPRA)
        message.append(self.rSPA)
        message.append(self.rFRA)
        message.append(self.rPRB)
        message.append(self.rSPB)
        message.append(self.rFRB)
        message.append(self.rPRC)
        message.append(self.rSPC)
        message.append(self.rFRC)
        message.append(self.rPRS)
        message.append(self.rSPS)
        message.append(self.rFRS)

        return message


    def tickleForceCommand(self, direction):
        if direction == 'up':
            self.rFRA +=1
            self.rFRB +=1
            self.rFRC +=1
            self.rFRS +=1

        if direction == 'down':
            self.rFRA -=1
            self.rFRB -=1
            self.rFRC -=1
            self.rFRS -=1

class statusModbusToLcmConverter(object):

    def __init__ (self):

        #Byte 0 - Gripper Status
        self.gACT = 0  # Activated: 1 bit, 0=reset, 1=activated
        self.gMOD = 0  # Mode 2 bits, 00=basic, 10=pinch, 01=wide, 11=scissor
        self.gGTO = 0  # Goto: 1 bit, 0=standby, 1=goto position request
        self.gIMC = 0  # Initialization & Mode Status: 2 bits, 00=reset, 10=activate in progress, 01=mode change, 11=complete
        self.gSTA = 0  # Gripper status: 2 bits, 00=moving, 10=one stopped, 01=all stopped, 11=all achieved

        #Byte 1 - Object Status
        # 00 - finger in motion
        # 10 - finger stopped while opening
        # 01 - finger stopped while closing
        # 11 - finger reached position request
        self.gDTA = 0  # Status finger A, 2 bits, see above
        self.gDTB = 0  # Status finger B, 2 bits, see above
        self.gDTC = 0  # Status finger C, 2 bits, see above
        self.gDTS = 0  # Status scissor axis, 2 bits, see above

        #Byte 2 - Fault Status
        # 0x00 - no fault
        # Priority
        # 0x05 - action delayed, activation must complete
        # 0x06 - action delayed, mode change must complete
        # 0x07 - activation bit must be set first
        # Minor
        # 0x09 - Comm no tready
        # 0x0A - Mode change fault, interfereance on scissor (<20s)
        # 0x0B - auto release in progress
        # Major
        # 0x0D - Action fault
        # 0x0E - change mode fault, interferance on scissor (>20s)
        # 0x0F - automatic release complete, reset and activate required
        self.gFLT = 0  # Fault: 4 bits, fault code (see above)
        self.gRS1 = 0  # Reserved, 4 bits, all zero

        #Byte 3
        self.gPRA = 0  # Echo of position request for finger A
        self.gPOA = 0  # Position of finger A
        self.gCUA = 0  # Current on finger A

        #Byte 3
        self.gPRB = 0  # Echo of position request for finger B
        self.gPOB = 0  # Position of finger B
        self.gCUB = 0  # Current on finger B

        #Byte 3
        self.gPRC = 0  # Echo of position request for finger C
        self.gPOC = 0  # Position of finger C
        self.gCUC = 0  # Current on finger C

        #Byte 3
        self.gPRS = 0  # Echo of position request for Scissor axis
        self.gPOS = 0  # Position of finger S
        self.gCUS = 0  # Current on finger S


    def populateStatus(self, rawStatData):
        #Populate internal variables by extracting bytes form the raw data

        if len(rawStatData) == 0:
            #There was an error reading, do nothing and just republish current status
            return

        self.gACT = (rawStatData[0] >> 0) & 0x01;
        self.gMOD = (rawStatData[0] >> 1) & 0x03;
        self.gGTO = (rawStatData[0] >> 3) & 0x01;
        self.gIMC = (rawStatData[0] >> 4) & 0x03;
        self.gSTA = (rawStatData[0] >> 6) & 0x03;
        self.gDTA = (rawStatData[1] >> 0) & 0x03;
        self.gDTB = (rawStatData[1] >> 2) & 0x03;
        self.gDTC = (rawStatData[1] >> 4) & 0x03;
        self.gDTS = (rawStatData[1] >> 6) & 0x03;
        self.gFLT =  rawStatData[2]
        self.gPRA =  rawStatData[3]
        self.gPOA =  rawStatData[4]
        self.gCUA =  rawStatData[5]
        self.gPRB =  rawStatData[6]
        self.gPOB =  rawStatData[7]
        self.gCUB =  rawStatData[8]
        self.gPRC =  rawStatData[9]
        self.gPOC =  rawStatData[10]
        self.gCUC =  rawStatData[11]
        self.gPRS =  rawStatData[12]
        self.gPOS =  rawStatData[13]
        self.gCUS =  rawStatData[14]

    def generateLcmStatus(self):
        #Use internal variables to populate an lcm message
        message = robotiqhand.status_t()

        # Status and fualt codes
        message.activated = self.gACT
        message.mode = self.gMOD
        message.gotoStatus = self.gGTO
        message.modeChangeStatus = self.gIMC
        message.motionStatus = self.gSTA
        message.faultCode = self.gFLT

        # Finger status codes
        message.fingerAStatus = self.gDTA
        message.fingerBStatus = self.gDTB
        message.fingerCStatus = self.gDTC
        message.fingerSStatus = self.gDTC

        # Motor position command
        message.posRequestA = self.gPRA
        message.posRequestB = self.gPRB
        message.posRequestC = self.gPRC
        message.posRequestS = self.gPRS

        # Motor position
        message.positionA = self.gPOA
        message.positionB = self.gPOB
        message.positionC = self.gPOC
        message.positionS = self.gPOS

        # Motor Current
        message.currentA = self.gCUA
        message.currentB = self.gCUB
        message.currentC = self.gCUC
        message.currentS = self.gCUS

        return message

class robotiqBaseSModel(object):
    """Base class (communication protocol agnostic) for sending
    commands and receiving the status of the Robotic S-Model
    gripper."""

    def __init__(self):

        #Initiate output message as an empty list
        self.message1 = []
        self.message2 = []

        #create and store local converter objects
        self.command = commandLcmToModbusConverter()
        self.status = statusModbusToLcmConverter()

        #Note: after the instantiation, a ".client" member must be added to the object


    def refreshCommand(self, channel, message):
        """Function to update the command which will be sent during
        the next sendCommand() call."""

        #Decode the lcm string into a message
        rawLcm = robotiqhand.command_t.decode(message)

        self.command.parseLcm(rawLcm)

        #Grab the message from the lcm converter
        self.command.tickleForceCommand('down')
        self.message1 = self.command.getModbusString()
        self.command.tickleForceCommand('up')
        self.message2 = self.command.getModbusString()


    def sendCommand(self):
        """Send the command to the Gripper."""

        # send the first desired command, then the modified one
        # this is done to trick the gripper into thinking all
        # commands are new
        # each sendCommand will send multiple times to try to overcome dropouts
        try:
            self.client.sendCommand(self.message1)
            self.client.sendCommand(self.message2)
        except AttributeError:
            print "Client not available.  Not sending command."

    def getStatus(self):
        """Request the status from the gripper and return it in the SModel_robot_input msg type."""

        if not self.client:
            return

        #Acquire status from the Gripper
        rawStatData = self.client.getStatus(16)

        if rawStatData == []:
            return

        #Use converter object to get the data
        self.status.populateStatus(rawStatData)
        message = self.status.generateLcmStatus()

        #Add a time stamp for this message
        message.utime = (time.time() * 1000000)

        return message
