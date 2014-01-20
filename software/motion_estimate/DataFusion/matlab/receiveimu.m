

%% set up the LCM variables
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('IMU_DATA', aggregator);

%set up a common message variable
posemsg = drc.nav_state_t();
posemsg.pose = drc.position_3d_t();
posemsg.pose.translation = drc.vector_3d_t();
posemsg.pose.rotation = drc.quaternion_t();
posemsg.twist = drc.twist_t();
posemsg.twist.linear_velocity = drc.vector_3d_t();
posemsg.twist.angular_velocity = drc.vector_3d_t();
posemsg.local_linear_acceleration = drc.vector_3d_t();

% the initial conditions for the system
RecursiveData.pose = init_pose();
RecursiveData.pose__k1 = init_pose();
RecursiveData.pose__k2 = init_pose();
% The recursive compensation data buffer structure
RecursiveData.INSCompensator = init_INSCompensator();

while true
    while true
        %disp waiting
        millis_to_wait = 1;
        msg = aggregator.getNextMessage(millis_to_wait);
        if length(msg) > 0
            break
        end
    end

%     disp(sprintf('channel of received message: %s', char(msg.channel)))
    %disp(sprintf('raw bytes of received message:'))
    %disp(sprintf('%d ', msg.data'))

    m = drc.imu_t(msg.data);

%     disp(sprintf('decoded message:\n'))
%     disp([ 'received timestamp:   ' sprintf('%d ', m.utime) ])
    
    % propagate the estimated navigation solution
    RecursiveData.pose = handle_imu(RecursiveData, m);
    % broadcast the estimated state via LCM
    sendpose_lcm(lc, posemsg, RecursiveData.pose);
    
    % Store previous states for INS mechanization
    RecursiveData.pose__k2 = RecursiveData.pose__k1;
    RecursiveData.pose__k1 = RecursiveData.pose;

    if (m.utime==(20000*1000))
        break
    end
end
