import numpy

# deprecated
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

# deprecated
def createStats(msgs):
  data = { 'UTIME' : [] }
  for msg in msgs:
    zipped = zip(msg.sent_channels,msg.sent_bytes)
    data['UTIME'].append(msg.utime)
    for pair in zipped:
      if pair[0] not in data:
        data[pair[0]] = [pair[1]]
      else:
        data[pair[0]].append(pair[1])
  for chan,dat in data.iteritems():
    data[chan] = numpy.array(dat)
  return data

def readBandwidthCsv(fileName):
  import csv
  with open(fileName, 'rb') as csvfile:
    reader = csv.DictReader(csvfile)
    data = {}
    for row in reader:

      # check if row is corrupt
      bad = False
      for key,val in row.iteritems():
        if type(val) is list:
          bad = True
          break
      if bad:
        continue
        
      for key,val in row.iteritems():
        val = int(val)
        if key not in data:
          data[key] = [val]
        else:
          data[key].append(val)
  for chan,dat in data.iteritems():
    data[chan] = numpy.array(dat)

  data0 = {'UTIME':data['UTIME']}
  data1 = {'UTIME':data['UTIME']}
  for chan,dat in data.iteritems():
    if (chan[-1] == '0'):
      data0[chan[0:-1]] = dat
    elif (chan[-1] == '1'):
      data1[chan[0:-1]] = dat

  return data0,data1
    

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
  times = data['UTIME']
  dt = float(times[-1] - times[0])/1e6
  allBytes = 0
  for chan,dat in data.iteritems():
    if (chan == 'UTIME'):
      continue
    totalBytes = dat[-1] - dat[0]
    totalVal,totalSuffix = getBandwidthString(totalBytes)
    avgVal,avgSuffix = getBandwidthString(totalBytes*8/dt)
    allBytes += totalBytes
    print('%s : %.3f %sbytes total, %.3f %sbps average' % (chan, totalVal, totalSuffix, avgVal, avgSuffix))
  val,suff = getBandwidthString(allBytes/dt)
  print ('IN TOTAL: %.3f %sbps average' % (val,suff))

def doPlots(data,title):
  import matplotlib.pyplot as plt
  times = data['UTIME']
  times = (times-times[0])/1e6
  dt = numpy.diff(times)
  channels = [chan for chan in data.keys() if chan != 'UTIME']
  legendChannels = []
  fig = plt.figure()
  plt.hold(True)
  totals = numpy.zeros(dt.shape)
  for chan in channels:
    dat = data[chan]*8
    ddat = numpy.diff(dat)/dt
    if numpy.any(ddat>1024):
      legendChannels.append(chan)
      plt.plot(times[1:],ddat/1024.0,linewidth=3)
    totals += ddat/1024.0
  plt.plot(times[1:],totals,linewidth=3,linestyle='dotted')
  legendChannels.append('TOTAL')
  plt.grid('on')
  plt.xlabel('time (s)')
  plt.ylabel('bandwidth (Kbits/sec)')
  plt.legend(legendChannels)
  plt.title(title + ' message bandwidths')
  return fig
  
def go(csvFile):
  import matplotlib.pyplot as plt
  import os.path
  (dataIn,dataOut) = readBandwidthCsv(csvFile)
  onRobot = 'usage-robot' in csvFile
  if onRobot:
    label = 'robot'
    (dataIn,dataOut) = (dataOut,dataIn)
  else:
    label = 'base'
  fileBase = os.path.splitext(csvFile)[0]
  fig = doPlots(dataIn, label+': received')
  fig.set_size_inches(10,8)
  plt.savefig(fileBase+'_'+label+'_rx.png', bbox_inches='tight', dpi=100)
  plt.close(fig)
  fig = doPlots(dataOut, label+': transmitted')
  fig.set_size_inches(10,8)
  plt.savefig(fileBase+'_'+label+'_tx.png', bbox_inches='tight', dpi=100)
  plt.close(fig)
  return (dataIn, dataOut)

def processDir(dirName):
  import glob
  import time
  for d in glob.glob(dirName + '/*.csv'):
    print('processing %s...' % (d))
    go(d)
x
