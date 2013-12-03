import bot_core
import lcm
import urllib2
import time
import sys
import os
import select
import subprocess
import threading


# VLC command:
# cvlc <input> --sout '#transcode{acodec=vorb,ab=10,channels=1,samplerate=8000}:std{access=http,mux=ogg,url=localhost:8080}'
# where <input> is a file or a url


serverChannel = 'OGG_SERVER'
clientChannel = 'OGG_CLIENT'
oggUrl = 'http://localhost:8080'
messageSize = 4096

serverThreadRunning = False
serverThread = None

def serverStreamLoop():

    stream = urllib2.urlopen(oggUrl)

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


def server():

    lcmHandle = lcm.LCM()
    subscription = lcmHandle.subscribe(clientChannel, handleMessageFromClient)
    while True:
        lcmHandle.handle()

    

oggProc = None

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
        server()
    else:
        client()

if __name__ == '__main__':
    main()
