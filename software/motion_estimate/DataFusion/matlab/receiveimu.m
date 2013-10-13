

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
pose.utime = 0;
pose.P = zeros(3,1);
pose.V = zeros(3,1);
pose.R = eye(3);
pose.f_l = zeros(3,1);

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
    pose = handle_imu(pose, m);
    % broadcast the estimated state via LCM
    sendpose_lcm(lc, posemsg, pose);
    

    if (m.utime==10000000)
        break
    end
end
