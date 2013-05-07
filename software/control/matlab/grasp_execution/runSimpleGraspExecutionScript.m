function runSimpleGraspExecutionScript()
megaclear
[r_left,r_right] = createSandiaManips();

nq_l = r_left.getNumStates/2;
l_jointNames = r_left.getStateFrame.coordinates(1:nq_l);
nq_r = r_right.getNumStates/2;
r_jointNames = r_right.getStateFrame.coordinates(1:nq_r);

lcmcoder = JLCMCoder(GraspStateCoder('atlas',l_jointNames,r_jointNames));
nx=18+nq_l+nq_r;

channel = ['COMMITTED_GRASP_SEED'];
disp(channel);
grasp_state_listener=LCMCoordinateFrameWCoder('atlas',nx,'x',lcmcoder);
setDefaultChannel(grasp_state_listener,channel);
grasp_state_listener.subscribe(channel);
%defaultChannel(grasp_opt_listener)


floating =false;
l_jointNames = r_left.getStateFrame.coordinates(1:nq_l);
r_jointNames = r_right.getStateFrame.coordinates(1:nq_r);

Kp = 5*[15  10  10   15  10  10    15  10  10 15  10  10]';
Kd = [1.5 0.9 0.6 1.5 0.9 0.6   1.5 0.9 0.6 1.5 0.9 0.6]';

l_coder = JLCMCoder(SandiaJointCommandCoder('atlas',floating,'left', l_jointNames,Kp,Kd));
l_hand_joint_cmd_publisher=LCMCoordinateFrameWCoder('sandia_left',4*nq_l,'q',l_coder);
r_coder = JLCMCoder(SandiaJointCommandCoder('atlas',floating,'right', r_jointNames,Kp,Kd));
r_hand_joint_cmd_publisher=LCMCoordinateFrameWCoder('sandia_right',4*nq_r,'q',r_coder);

Kp2 = [ 15  0 0    15  0 0    15 0 0    15  10  10]';
Kd2 = [1.5 0 0    1.5 0 0   1.5 0 0   1.5 0.9 0.6]';
pos_control_flag = [1.0 0 0    1.0 1.0 1.0   1.0 0 0   1.0 1.0 1.0]';
%Kd(find(pos_control_flag==0))= 0;
%Kp(find(pos_control_flag==0))= 10;
while(1)
    [x,ts] = getNextMessage(grasp_state_listener,1);%getNextMessage(obj,timeout)
    if (~isempty(x))
        fprintf('received message at time %f\n',ts);
        %fprintf('state is %f\n',x);
        
        msg = grasp_state_listener.lcmcoder.encode(ts,x);

%         rpy = quat2rpy([x(9);x(6:8)]);   
%         q_l = [x(3:5);rpy(1);rpy(2);rpy(3);msg.l_joint_position];   
%         [r,p,y] = quat2angle([x(16);x(13:15)]','XYZ');
%         rpy = quat2rpy([x(16);x(13:15)]); 
%         q_r = [x(10:12);rpy(1);rpy(2);rpy(3);msg.r_joint_position];%-sign(msg.r_joint_position)*0.01
%         q_l(1) =q_l(1)+0.1;
%         q_r(1) =q_r(1)-0.1;

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
        torque =2*0.2;
        if(sum(msg.l_joint_position)>0)
           e_l(find(pos_control_flag==0)) = torque;   
%            K_pos(find(pos_control_flag==0))= 0;
%            K_vel(find(pos_control_flag==0))= 0;
        elseif(sum(msg.r_joint_position)>0) 
%            torque =1.0;
%            e_r(find(pos_control_flag==0)) = torque;
%             K_pos(find(pos_control_flag==0))= 0;
%             K_vel(find(pos_control_flag==0))= 0;
      
           
           e_r(find(pos_control_flag==0)) = torque;            
        end
        
        if(msg.grasp_type==msg.SANDIA_LEFT)
            publish(l_hand_joint_cmd_publisher,ts,[K_pos;K_vel;q_l;e_l]','L_HAND_JOINT_COMMANDS');
        elseif(msg.grasp_type==msg.SANDIA_RIGHT)
            publish(r_hand_joint_cmd_publisher,ts,[K_pos;K_vel;q_r;e_r]','R_HAND_JOINT_COMMANDS');
        end
        
              
%         if(sum(msg.r_joint_position)>0) %do mixed torque control
%             fprintf('torque control\n');
%             for(i=1:length(Kp2))
%                if (Kp2(i)==0),
%                  q_l(6+i)=1;
%                  q_r(6+i)=1;
%                end
%             end
%             if(msg.grasp_type==msg.SANDIA_LEFT)
%                 publish(l_hand_joint_cmd_publisher,ts,[Kp2;Kd2;q_l]','L_HAND_JOINT_COMMANDS');
%             elseif(msg.grasp_type==msg.SANDIA_RIGHT)
%                 publish(r_hand_joint_cmd_publisher,ts,[Kp2;Kd2;q_r]','R_HAND_JOINT_COMMANDS');
%             end
%         else
%             if(msg.grasp_type==msg.SANDIA_LEFT)
%                 publish(l_hand_joint_cmd_publisher,ts,[Kp;Kd;q_l]','L_HAND_JOINT_COMMANDS');
%             elseif(msg.grasp_type==msg.SANDIA_RIGHT)
%                 publish(r_hand_joint_cmd_publisher,ts,[Kp;Kd;q_r]','R_HAND_JOINT_COMMANDS');
%             end
%             
%         end
        
   
        
    end
end %end while


end

