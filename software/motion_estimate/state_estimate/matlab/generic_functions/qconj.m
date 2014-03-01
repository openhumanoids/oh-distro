function q = qconj(q_)
% quaternion conjugate
% quaternions are scalar vector: [w;x;y;z]

q = q_;
q(2:4) = -q_(2:4);
