import numpy

def readAllBandwidthMessages(fileName,channel):
  import lcm
  from drc import bandwidth_stats_t
  
  log = lcm.EventLog(fileName)
  msgs = []
  for event in log:
    if event.channel == channel:
      msg = bandwidth_stats_t.decode(event.data)
      msgs.append(msg)
  print 'done reading log'
  return msgs

def createStats(msgs):
  data = { 'utime' : [] }
  for msg in msgs:
    zipped = zip(msg.sent_channels,msg.sent_bytes)
    data['utime'].append(msg.utime)
    for pair in zipped:
      if pair[0] not in data:
        data[pair[0]] = [pair[1]]
      else:
        data[pair[0]].append(pair[1])
  for chan,dat in data.iteritems():
    data[chan] = numpy.array(dat)
  return data

def getBandwidthString(numBytes):
  if int(numBytes/1024)==0:
    return (numBytes, '')
  elif int(numBytes/1024**2)==0:
    return (numBytes/1024.0, 'K')
  elif int(numBytes/1024**3)==0:
    return (numBytes/1024.0**2, 'M')
  else:
    return (numBytes/1024.0**3, 'G')

def printStats(data):
  times = data['utime']
  dt = float(times[-1] - times[0])/1e6
  allBytes = 0
  for chan,dat in data.iteritems():
    if (chan == 'utime'):
      continue
    totalBytes = dat[-1] - dat[0]
    totalVal,totalSuffix = getBandwidthString(totalBytes)
    avgVal,avgSuffix = getBandwidthString(totalBytes/dt)
    allBytes += totalBytes
    print('%s : %.3f %sbytes total, %.3f %sbps average' % (chan, totalVal, totalSuffix, avgVal, avgSuffix))
  val,suff = getBandwidthString(allBytes/dt)
  print ('IN TOTAL: %.3f %sbps average' % (val,suff))

def doPlots(data,title):
  import matplotlib.pyplot as plt
  times = data['utime']
  times = (times-times[0])/1e6
  dt = numpy.diff(times)
  channels = [chan for chan in data.keys() if chan != 'utime']
  legendChannels = []
  fig = plt.figure()
  plt.hold(True)
  totals = numpy.zeros(dt.shape)
  for chan in channels:
    dat = data[chan]
    ddat = numpy.diff(dat)/dt
    if numpy.any(ddat>1024):
      legendChannels.append(chan)
      plt.plot(times[1:],ddat/1024.0,linewidth=3)
    totals += ddat/1024.0
  plt.plot(times[1:],totals,linewidth=3)
  legendChannels.append('TOTAL')
  plt.grid('on')
  plt.xlabel('time (s)')
  plt.ylabel('bandwidth (Kbits/sec)')
  plt.legend(legendChannels)
  plt.title(title + ' message bandwidths')
  return fig
  
def go(fileNamePrefix):
  import matplotlib.pyplot as plt
  fileNameRobot = fileNamePrefix + '-robot'
  fileNameBase = fileNamePrefix + '-base'
  msgs = readAllBandwidthMessages(fileNameRobot, 'ROBOT_BW_STATS')
  dataRobot = createStats(msgs)
  msgs = readAllBandwidthMessages(fileNameBase, 'BASE_BW_STATS')
  dataBase = createStats(msgs)
  #printStats(data)
  fig = doPlots(dataRobot,'robot->base (downlink)')
  fig.set_size_inches(10,8)
  plt.savefig(fileNameRobot + '.png', bbox_inches='tight', dpi=100)
  fig = doPlots(dataBase, 'base->robot (uplink)')
  fig.set_size_inches(10,8)
  plt.savefig(fileNameBase + '.png', bbox_inches='tight', dpi=100)
  return (dataRobot, dataBase)
