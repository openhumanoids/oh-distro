function testedge_lcmlog
javaaddpath('/home/drc/drc/software/externals/libbot-drc/bot2-core/pod-build/lcmtypes_bot2-core.jar');
javaaddpath('/usr/local/share/java/lcm.jar');


%dirname = '/home/drc/drcdata/robotiq/2014-02-24-camera-rhand-tabletop';
%logfilename = 'lcmlog-2014-02-24-12-44-robot';
%CHANNELNAME = 'CAMERARHAND';
dirname = '/home/drc/drcdata/robotiq/2014-02-12-camera-tabletop-drillmove/';
logfilename = 'lcmlog-2014-02-12.00';
CHANNELNAME = 'DUMMY_CAMERA_CHANNEL';
logfile = fullfile(dirname, logfilename);
imagedir = '/tmp/tmppic/';
mkdir(imagedir);
lcmlog = lcm.logging.Log(logfile,'r');

close all
figure(1);

h_slider = uicontrol(1,'Style', 'slider', 'Min',0.0001,'Max',0.01, 'Value',0.003, 'Position',[0 0 200 20]);


h_button = uicontrol(1,'Style', 'pushbutton', 'String', 'Stop', 'Position',[200 0 40 20], 'Callback', {@buttonCallback});
global running;
running = true;
while (running)
    try
        event = lcmlog.readNext();
    catch ex
        break;
    end
    channel = char(event.channel);
    if (strcmp(channel,CHANNELNAME))
        img = bot_core.image_t(event.data);
        data = img.data;
        
        imgpath = sprintf('%stmp.jpg',imagedir);
        fid = fopen(imgpath, 'w');

        fwrite(fid, data, 'int8');
        fclose(fid);
        
        edgeandshow(imread(imgpath), get(h_slider,'Value'));
        pause(0.01)
    end
end
end

function buttonCallback(hObject, evt)
    global running;
    running = false;
end