function runGraspExecutionForIRobotHand()
megaclear
[r_left,r_right] = createIrobotManips();

nq_l = r_left.getNumStates/2;
l_jointNames = r_left.getStateFrame.coordinates(1:nq_l);
nq_r = r_right.getNumStates/2;
r_jointNames = r_right.getStateFrame.coordinates(1:nq_r);

lcmcoder = JLCMCoder(drc.control.GraspStateCoder('irobot',l_jointNames,r_jointNames));
nx=19+nq_l+nq_r;

channel = ['COMMITTED_GRASP'];
disp(channel);
grasp_state_listener=LCMCoordinateFrameWCoder('irobot',nx,'x',lcmcoder);
setDefaultChannel(grasp_state_listener,channel);
grasp_state_listener.subscribe(channel);
%defaultChannel(grasp_opt_listener)


floating =false;
l_jointNames = r_left.getStateFrame.coordinates(1:nq_l);
r_jointNames = r_right.getStateFrame.coordinates(1:nq_r);

Kp = [10 10 10 10 10 10 10 10]';
Kd = [1  1  1  1  1  1  1  1]';

l_coder = JLCMCoder(drc.control.IrobotJointCommandCoder('irobot',floating,'left', l_jointNames,Kp,Kd));
l_hand_joint_cmd_publisher=LCMCoordinateFrameWCoder('irobot_left',4*nq_l,'q',l_coder);
r_coder = JLCMCoder(drc.control.IrobotJointCommandCoder('irobot',floating,'right', r_jointNames,Kp,Kd));
r_hand_joint_cmd_publisher=LCMCoordinateFrameWCoder('irobot_right',4*nq_r,'q',r_coder);


pos_control_flag = [1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0]';% where ever there is zero we are doing mixed control in gazebo
msg_timeout = 5; % ms


init = false;
while(1)
    if (init==false)
        init=true;
        out_string = 'Irobot Grasp Controller: Ready'; disp(out_string); send_status(3,0,0,out_string);
    end
    
    [x,ts] = getNextMessage(grasp_state_listener,msg_timeout);%getNextMessage(obj,timeout)
    if (~isempty(x))
        out_string = 'Grasp: Got msg'; disp(out_string); send_status(3,0,0,out_string);
        fprintf('received message at time %f\n',ts);
        %fprintf('state is %f\n',x);
        
        msg = grasp_state_listener.lcmcoder.encode(ts,x);
        if(msg.power_grasp==1.0)
         pos_control_flag = [1.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0]';
         %pos_control_flag = [1.0 0 1.0    1.0 0.0 1.0   1.0 0 1.0   1.0 0.0 1.0]';
        end

        rpy = quat2rpy([x(9);x(6:8)]);
        l_hand_pose = [x(3:5);rpy(1);rpy(2);rpy(3)];
        rpy = quat2rpy([x(16);x(13:15)]); 
        r_hand_pose = [x(10:12);rpy(1);rpy(2);rpy(3)];
        
        q_l = [msg.l_joint_position];   
        q_r = [msg.r_joint_position];  
        
        K_pos=Kp;
        K_vel=Kd;
        e_l =q_l*0;
        e_r =q_r*0;
        e_l(find(pos_control_flag>0)) = 0;
        e_r(find(pos_control_flag>0)) = 0;
        if(msg.power_grasp==1.0)
          torque = 10;
          K_pos(find(pos_control_flag==0))= 0;
          K_vel(find(pos_control_flag==0))= 0;
        else
          torque =1;
        end
        if(sum(msg.l_joint_position)>0)
           e_l(find(pos_control_flag==0)) = torque;   
        elseif(sum(msg.r_joint_position)>0)
           e_r(find(pos_control_flag==0)) = torque; 
        end
        
        if(msg.grasp_type==msg.IROBOT_LEFT)
            publish(l_hand_joint_cmd_publisher,ts,[K_pos;K_vel;q_l;e_l]','L_HAND_JOINT_COMMANDS');
        elseif(msg.grasp_type==msg.IROBOT_RIGHT)
            publish(r_hand_joint_cmd_publisher,ts,[K_pos;K_vel;q_r;e_r]','R_HAND_JOINT_COMMANDS');
        elseif(msg.grasp_type==msg.IROBOT_BOTH)
            publish(l_hand_joint_cmd_publisher,ts,[K_pos;K_vel;q_l;e_l]','L_HAND_JOINT_COMMANDS');
            publish(r_hand_joint_cmd_publisher,ts,[K_pos;K_vel;q_r;e_r]','R_HAND_JOINT_COMMANDS');
        end

        
    end
end %end while


end

