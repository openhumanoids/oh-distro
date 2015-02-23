import lcm

def stripLog(inFile, outFile):
    channels = {'CAMERA':True, 'SCAN':True, 'PRE_SPINDLE_TO_POST_SPINDLE':True}
    
    inLog = lcm.EventLog(inFile, 'r')
    outLog = lcm.EventLog(outFile, 'w', overwrite=True)
    
    for event in inLog:
        if event.channel in channels:
            outLog.write_event(event.timestamp, event.channel, event.data)
    print 'done'

if __name__=='__main__':
    inFile = '/home/antone/data/2015-02-11_multisense-02-calib/lcmlog-2015-02-11.00'
    outFile = '/home/antone/data/2015-02-11_multisense-02-calib/lcmlog-2015-02-11.00.stripped'
    stripLog(inFile, outFile)

