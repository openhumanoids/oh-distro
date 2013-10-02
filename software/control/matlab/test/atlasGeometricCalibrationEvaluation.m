options.floating = true;
robot = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
rand('twister', 1253);

nq = getNumDOF(robot);
link_name = 'l_hand+l_hand_point_mass';
link_index = robot.findLinkInd(link_name);
point = [0; 0; 0];
rotation_type = 0; % no rotation included

n_measurements = 50;
calibration_index = 3; % number of equations per measurement
n_joints = 6; % number of joints on the path
W = zeros(n_measurements * calibration_index, n_joints);
condition_numbers = zeros(n_measurements, 1);

arm_joint_indices = 6 : 11;

for i = 0 : n_measurements
    q = rand(nq, 1) * pi - pi / 2;
    kinsol = robot.doKinematics(q);
    [~,J] = robot.forwardKin(kinsol, link_index, point, rotation_type);
    start_row = calibration_index * i + 1;
    W(start_row : start_row + calibration_index - 1, :) = J(:, arm_joint_indices);
    condition_numbers(i + 1) = cond(W);
end

plot(condition_numbers);
xlabel('number of measurements');
ylabel('condition number');

