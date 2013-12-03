import robotiqhand
import time

class robotiqBaseSModel:
    """Base class (communication protocol agnostic) for sending
    commands and receiving the status of the Robotic S-Model
    gripper."""

    def __init__(self):

        #Initiate output message as an empty list
        self.message = []

        #Note: after the instantiation, a ".client" member must be added to the object


    def verifyCommand(self, command):
        """Function to verify that the value of each variable satisfy
        its limits."""

        #Verify that each variable is in its correct range
        command.rACT = max(0, command.rACT)
        command.rACT = min(1, command.rACT)

        command.rMOD = max(0, command.rMOD)
        command.rMOD = min(3, command.rMOD)

        command.rGTO = max(0, command.rGTO)
        command.rGTO = min(1, command.rGTO)

        command.rATR = max(0, command.rATR)
        command.rATR = min(1, command.rATR)

        command.rGLV = max(0, command.rGLV)
        command.rGLV = min(1, command.rGLV)

        command.rICF = max(0, command.rICF)
        command.rICF = min(1, command.rICF)

        command.rICS = max(0, command.rICS)
        command.rICS = min(1, command.rICS)

        command.rPRA = max(0,   command.rPRA)
        command.rPRA = min(255, command.rPRA)

        command.rSPA = max(0,   command.rSPA)
        command.rSPA = min(255, command.rSPA)

        command.rFRA = max(0,   command.rFRA)
        command.rFRA = min(255, command.rFRA)

        command.rPRB = max(0,   command.rPRB)
        command.rPRB = min(255, command.rPRB)

        command.rSPB = max(0,   command.rSPB)
        command.rSPB = min(255, command.rSPB)

        command.rFRB = max(0,   command.rFRB)
        command.rFRB = min(255, command.rFRB)

        command.rPRC = max(0,   command.rPRC)
        command.rPRC = min(255, command.rPRC)

        command.rSPC = max(0,   command.rSPC)
        command.rSPC = min(255, command.rSPC)

        command.rFRC = max(0,   command.rFRC)
        command.rFRC = min(255, command.rFRC)

        command.rPRS = max(0,   command.rPRS)
        command.rPRS = min(255, command.rPRS)

        command.rSPS = max(0,   command.rSPS)
        command.rSPS = min(255, command.rSPS)

        command.rFRS = max(0,   command.rFRS)
        command.rFRS = min(255, command.rFRS)

        #Return the modified command
        return command

    def int_uint_convert(self, command):

        attr_list = [x for x in dir(command) if x[0]=='r']

        for attr in attr_list:
            x = command.__getattribute__(attr)
            if x < 0:
                command.__setattr__(attr,x+256)

        return command

    def refreshCommand(self, channel, message):
        """Function to update the command which will be sent during
        the next sendCommand() call."""

        #Decode the lcm string into a message
        command = robotiqhand.command_t.decode(message)

        #Unwrap the ints back into uints
        command = self.int_uint_convert(command)

        #Limit the value of each variable
        command = self.verifyCommand(command)

        #Initiate command as an empty list
        self.message = []

        #Build the command with each output variable
        self.message.append(command.rACT + (command.rMOD << 1) + (command.rGTO << 3) + (command.rATR << 4))
        self.message.append(command.rGLV + (command.rICF << 2) + (command.rICS << 3))
        self.message.append(0)
        self.message.append(command.rPRA)
        self.message.append(command.rSPA)
        self.message.append(command.rFRA)
        self.message.append(command.rPRB)
        self.message.append(command.rSPB)
        self.message.append(command.rFRB)
        self.message.append(command.rPRC)
        self.message.append(command.rSPC)
        self.message.append(command.rFRC)
        self.message.append(command.rPRS)
        self.message.append(command.rSPS)
        self.message.append(command.rFRS)

    def sendCommand(self):
        """Send the command to the Gripper."""

        self.client.sendCommand(self.message)

    def getStatus(self):
        """Request the status from the gripper and return it in the SModel_robot_input msg type."""

        #Acquire status from the Gripper
        status = self.client.getStatus(16);

        #Message to output
        message = robotiqhand.status_original_t()

        #Add a time stamp for this message
        message.utime = (time.time() * 1000000)

        if len(status) == 0:
            return []

        #Assign the values to their respective variables
        message.gACT = (status[0] >> 0) & 0x01;
        message.gMOD = (status[0] >> 1) & 0x03;
        message.gGTO = (status[0] >> 3) & 0x01;
        message.gIMC = (status[0] >> 4) & 0x03;
        message.gSTA = (status[0] >> 6) & 0x03;
        message.gDTA = (status[1] >> 0) & 0x03;
        message.gDTB = (status[1] >> 2) & 0x03;
        message.gDTC = (status[1] >> 4) & 0x03;
        message.gDTS = (status[1] >> 6) & 0x03;
        message.gFLT =  status[2]
        message.gPRA =  status[3]
        message.gPOA =  status[4]
        message.gCUA =  status[5]
        message.gPRB =  status[6]
        message.gPOB =  status[7]
        message.gCUB =  status[8]
        message.gPRC =  status[9]
        message.gPOC =  status[10]
        message.gCUC =  status[11]
        message.gPRS =  status[12]
        message.gPOS =  status[13]
        message.gCUS =  status[14]

        return message
