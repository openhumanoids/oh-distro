import lcm
import time
import StringIO

class LCMWrapper(object):
    '''
    A basic class providing some convenience methods around LCM.
    '''

    def __init__(self):
        self.lc = lcm.LCM()

    def subscribe(self, channel, callback, messageClass=None):
        if messageClass is not None:
            def handleMessage(channel, messageData):
                callback(channel, messageClass.decode(messageData))
            sub = self.lc.subscribe(channel, handleMessage)
        else:
            sub = self.lc.subscribe(channel, callback)
        return sub

    def publish(self, channel, message):
        self.lc.publish(channel, message.encode())

    def captureMessages(self, channel, messageClass, numberOfMessages):

        messages = []
        def handleMessage(channel, messageData):
            messages.append(messageClass.decode(messageData))

        subscription = self.subscribe(channel, handleMessage)

        while len(messages) < numberOfMessages:
            self.lc.handle()

        self.lc.unsubscribe(subscription)
        return messages

    def captureMessage(self, channel, messageClass):
        return captureMessages(channel, messageClass, 1)[0]

    def startHandleLoop(self):
        while True:
            self.lc.handle()


lcmHandle = LCMWrapper()

def getLCM():
    return lcmHandle

def getMessageFingerprint(data):
    if hasattr(data, 'read'):
        buf = data
    else:
        buf = StringIO.StringIO(data)
    return buf.read(8)
