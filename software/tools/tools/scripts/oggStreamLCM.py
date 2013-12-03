import bot_core
import lcm
import urllib2
import time
import sys
import os
import select
import subprocess
import threading

#
# VLC example command for robot computer to Atlas D-Link webcam:
#
# cvlc rtsp://10.66.171.26:554/live2.sdp --sout '#transcode{acodec=vorb,ab=10,channels=1,samplerate=8000}:std{access=http,mux=ogg,url=localhost:8123}'
#

serverChannel = 'OGG_SERVER'
clientChannel = 'OGG_CLIENT'
oggUrl = 'localhost:8080'
webcamUrl = 'rtsp://10.66.171.26:554/live2.sdp'
transcodeArgs = 'ab=10,channels=1,samplerate=8000'
messageSize = 4096

serverThreadRunning = False
serverThread = None
vlcProc = None
oggProc = None

def serverStreamLoop():

    stream = urllib2.urlopen('http://'+oggUrl)

    lcmHandle = lcm.LCM()
    m = bot_core.raw_t()
    m.utime = 0
    totalBytes = 0

    global serverThreadRunning
    while serverThreadRunning:

        m.data = stream.read(messageSize)
        if not m.data:
            break

        m.utime = m.utime + 1
        m.length = len(m.data)
        totalBytes += m.length

        #print 'publishing message %d. %d bytes. total so far: %f kB' % (m.utime, m.length, totalBytes/1024.0)

        lcmHandle.publish(serverChannel, m.encode())

    print 'stream publisher loop returning'


def handleMessageFromClient(channel, data):

    m = bot_core.raw_t.decode(data)
    print 'message from client:', m.data

    global serverThread, serverThreadRunning

    if serverThread:
        serverThreadRunning = False
        serverThread.join()
        serverThread = None

    serverThreadRunning = True
    serverThread = threading.Thread(target=serverStreamLoop)
    serverThread.daemon = True
    serverThread.start()


def startVLC(vlcInputStream):

    soutString = '#transcode{acodec=vorb,%s}:std{access=http,mux=ogg,url=%s}' % (transcodeArgs, oggUrl)
    command = ['cvlc', vlcInputStream, '--sout', soutString]

    print 'starting VLC with command:'
    print ' '.join(command)

    global vlcProc
    vlcProc = subprocess.Popen(command)


def server(vlcInputStream):

    startVLC(vlcInputStream)

    lcmHandle = lcm.LCM()
    subscription = lcmHandle.subscribe(clientChannel, handleMessageFromClient)
    while True:
        lcmHandle.handle()


def handleMessageFromServer(channel, data):
    m = bot_core.raw_t.decode(data)
    oggProc.stdin.write(m.data)


def client():

    global oggProc
    oggProc = subprocess.Popen(['ogg123', '-'], stdin=subprocess.PIPE, stderr=subprocess.STDOUT)

    lcmHandle = lcm.LCM()

    m = bot_core.raw_t()
    m.utime = 0
    m.data = 'restart_stream'
    m.length = len(m.data)
    lcmHandle.publish(clientChannel, m.encode())

    subscription = lcmHandle.subscribe(serverChannel, handleMessageFromServer)
    while True:
        lcmHandle.handle()


def main():

    mode = sys.argv[1]
    assert mode in ('--client', '--server')

    if mode == '--server':
        try:
            vlcInputStream = sys.argv[2]
        except IndexError:
            vlcInputStream = webcamUrl
        server(vlcInputStream)
    else:
        client()

if __name__ == '__main__':
    main()
