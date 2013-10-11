function pose = receivepose(aggregator)


while true
    %disp waiting
    millis_to_wait = 1;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        break
    end
end

m = drc.nav_state_t(msg.data);

% disp(['got it ' num2str(m.utime)]);

% push the data back into a MATLAB structure for evaluation relative to
% ground_truth data
pose.utime = m.utime;
pose.P = [m.pose.translation.x;m.pose.translation.y;m.pose.translation.z];
pose.V = [m.twist.linear_velocity.x;m.twist.linear_velocity.y;m.twist.linear_velocity.z];
pose.q = [m.pose.rotation.w;m.pose.rotation.x;m.pose.rotation.y;m.pose.rotation.z];
pose.w_l = [m.twist.angular_velocity.x;m.twist.angular_velocity.y;m.twist.angular_velocity.z];
pose.f_l = [m.local_linear_acceleration.x;m.local_linear_acceleration.y;m.local_linear_acceleration.z];

% pose.R = q2R(pose.q);

