function out = interpolate_pose(in1,in2,timestamp)

f = double(timestamp-in1.timestamp)/double(in2.timestamp-in1.timestamp);
out.T = (1-f)*in1.T(:) + f*in2.T(:);
q1 = rot2quat(in1.R);
q2 = rot2quat(in2.R);
q = (1-f)*q1 + f*q2;
out.R = quat2rot(q);
out.timestamp = timestamp;
