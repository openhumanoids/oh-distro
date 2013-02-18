function runSimpleGraspExecutionScript()
megaclear
[r_left,r_right] = createSandiaManips();

nq_l = r_left.getNumStates/2;
l_jointNames = r_left.getStateFrame.coordinates(7:nq_l);
nq_r = r_right.getNumStates/2;
r_jointNames = r_right.getStateFrame.coordinates(7:nq_r);

% Temporary fix as URDFs for sticky hands are not up to date. TODO: remove later
l_jointNames = regexprep(l_jointNames,'left_','l_');
r_jointNames = regexprep(r_jointNames,'right_','r_');

lcmcoder = JLCMCoder(GraspStateCoder('atlas',l_jointNames,r_jointNames));
nx=18+nq_l-6+nq_r-6; % this changes?

channel = ['COMMITTED_GRASP_SEED'];
disp(channel);
grasp_state_listener=LCMCoordinateFrameWCoder('atlas',nx,'x',lcmcoder);
setDefaultChannel(grasp_state_listener,channel);
grasp_state_listener.subscribe(channel);
%defaultChannel(grasp_opt_listener)


floating =true;
l_jointNames_w_floating = r_left.getStateFrame.coordinates(1:nq_l);
r_jointNames_w_floating = r_right.getStateFrame.coordinates(1:nq_r);
Kp = [80 80 80   40 40 5    15 10 10    15 10 10    15 10 10    15 10 10   ]';
Kd = [ 34 34 34  14 14 2.5  1.5 0.9 0.6 1.5 0.9 0.6 1.5 0.9 0.6 1.5 0.9 0.6]';
l_coder = JLCMCoder(SandiaJointCommandCoder('atlas',floating,'left', l_jointNames_w_floating,Kp,Kd));
l_hand_joint_cmd_publisher=LCMCoordinateFrameWCoder('sandia_left',nq_l,'q',l_coder);
r_coder = JLCMCoder(SandiaJointCommandCoder('atlas',floating,'right', r_jointNames_w_floating,Kp,Kd));
r_hand_joint_cmd_publisher=LCMCoordinateFrameWCoder('sandia_right',nq_r,'q',r_coder);

while(1)
    [x,ts] = getNextMessage(grasp_state_listener,1);%getNextMessage(obj,timeout)
    if (~isempty(x))
        fprintf('received message at time %f\n',ts);
        %fprintf('state is %f\n',x);
        
        msg = grasp_state_listener.lcmcoder.encode(ts,x);
        %   n_l_joints =nq_l-6;
        %   n_r_joints =nq_r-6;
        %   l_joint_positions = x(19:19+n_l_joints-1);
        %   r_joint_positions = x(19+n_l_joints:19+n_l_joints+n_r_joints-1);
        
        [r,p,y] = quat2angle(x(6:9)','ZYX');
        q_l = [x(3:5);r;p;y;msg.l_joint_position];
        [r,p,y] = quat2angle(x(13:16)','ZYX');
        q_r = [x(10:12);r;p;y;msg.r_joint_position];
        if(msg.grasp_type==msg.SANDIA_LEFT)
            publish(l_hand_joint_cmd_publisher,ts,q_l,'L_HAND_JOINT_COMMANDS');
        elseif(msg.grasp_type==msg.SANDIA_RIGHT)
            publish(r_hand_joint_cmd_publisher,ts,q_r,'R_HAND_JOINT_COMMANDS');
        end
        
        
    end
end %end while


end

