javaaddpath('/home/drc/drc/software/externals/libbot-drc/bot2-core/pod-build/lcmtypes_bot2-core.jar');
javaaddpath('/usr/local/share/java/lcm.jar');


dirname = '/home/drc/drcdata/robotiq/2014-02-24-camera-rhand-tabletop';
logfilename = 'lcmlog-2014-02-24-12-42-robot';
logfile = fullfile(dirname, logfilename);
imagedir = '/tmp/tmppic/';
mkdir(imagedir);
lcmlog = lcm.logging.Log(logfile,'r');


while (true)
    try
        event = lcmlog.readNext();
    catch ex
        break;
    end
    channel = char(event.channel);
    if (strcmp(channel,'CAMERARHAND'))
        img = bot_core.image_t(event.data);
        data = img.data;
        
        imgpath = sprintf('%stmp.jpg',imagedir);
        fid = fopen(imgpath, 'w');

        fwrite(fid, data, 'int8');
        fclose(fid);
        
        edgeandshow(imread(imgpath));
        pause(0.01)
    end
end