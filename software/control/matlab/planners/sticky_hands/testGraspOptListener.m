function testGraspOptListener()

lcmcoder = JLCMCoder(drc.control.GraspSeedOptCoder('atlas'));

channel = ['INIT_GRASP_SEED_OPT_1'];
disp(channel);
grasp_opt_listener=LCMCoordinateFrame('atlas',lcmcoder,'x');
setDefaultChannel(grasp_opt_listener,channel);
grasp_opt_listener.subscribe(channel);
%defaultChannel(grasp_opt_listener)

while(1)
    [x,ts] = getNextMessage(grasp_opt_listener,1);%getNextMessage(obj,timeout)
    if (~isempty(x))
        fprintf('received message at time %f\n',ts);
        fprintf('state is %f\n',x);
        %msg = grasp_opt_listener.lcmcoder.encode(ts,x);
    end
end %end while


end

