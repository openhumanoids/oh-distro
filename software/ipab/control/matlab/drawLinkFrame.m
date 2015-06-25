function lcmClient = drawLinkFrame(robot, link, state, name)
    if nargin < 4
        name = linkName;
    end
    lcmClient = LCMGLClient(name);
    robot.drawLCMGLAxes(lcmClient, state, link);
    lcmClient.switchBuffers();
end