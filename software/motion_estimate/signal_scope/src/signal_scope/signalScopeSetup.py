import lcm
import drc
import bot_core



messageTypes = {}
signals = []

def loadMessageTypes(typesDict, typesModule):

    originalSize = len(typesDict)
    for name, value in typesModule.__dict__.iteritems():
        if hasattr(value, '_get_packed_fingerprint'):
            typesDict[value._get_packed_fingerprint()] = value

    print 'Loaded %d lcm message types from: %s' % (len(typesDict) - originalSize, typesModule.__name__)

loadMessageTypes(messageTypes, drc)
loadMessageTypes(messageTypes, bot_core)


class LookupHelper(object):

  def __init__(self, lookups=()):
      self._lookups = lookups

  def __getitem__(self, i):
      return LookupHelper(self._lookups + (i,))

  def __getattr__(self, attr):
      return LookupHelper(self._lookups + (attr,))


msg = LookupHelper()

def decodeMessageFunction(messageBytes):

    s = str(messageBytes)
    message = messageTypes[s[:8]].decode(s)
    return message



def decodeMessageHelper(func):

    def decodeMessage(messageBytes):
        s = str(messageBytes)
        message = messageTypes[s[:8]].decode(s)
        return func(message)

    return decodeMessage


def addSignal(channel, func):

    if isinstance(func, LookupHelper):
        addSignalHelper(channel, func)
        return

    def funcWithUtime(msg):
        return msg.utime, func(msg)

    signals.append((channel, funcWithUtime))


def addSignalHelper(channel, lookups):

    lookups = lookups._lookups

    def func(msg):

        value = msg
        for field in lookups:
            value = getattr(value, field) if isinstance(field, str) else value[field]
        return msg.utime, value

    addSignal(channel, func)
