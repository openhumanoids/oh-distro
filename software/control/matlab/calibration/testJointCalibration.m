function testJointCalibration
% NOTEST
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
joint_indices = 21:26;
body1 = 5; %utorso
body2 = 29;
N = 9;
q_data = zeros(34,N);
q_data(1:6,:) = randn(6,N);
q_data(joint_indices,:) = randn(6,N);
body1_data = zeros(3,3,N);
body2_data = zeros(3,3,N);

params1 = randn(3,1);
params2 = randn(3,1);

pts_body1 = body1_test_fun(params1);
pts_body2 = body1_test_fun(params2);

for i=1:N,
  kinsol=r.doKinematics(q_data(:,i));
  body1_data(:,:,i) = r.forwardKin(kinsol,body1,pts_body1);
  body2_data(:,:,i) = r.forwardKin(kinsol,body2,pts_body2);
end



[dq, body1_params, body2_params, floating_states, residuals, info] = jointOffsetCalibration(r, q_data, joint_indices,body1,@body1_test_fun, 3, body1_data, body2, @body2_test_fun, 3, body2_data);

keyboard
end

function [x,dx] = body1_test_fun(params)
  x = [[0;.1;0] [0;-.1;0] [0;0;.1]] + repmat(params,1,3);
  dx = repmat(eye(3),3,1);
end

function [x,dx] = body2_test_fun(params)
  x = [[0;.1;0] [0;-.1;0] [0;0;.1]] + repmat(params,1,3);
  dx = repmat(eye(3),3,1);
end