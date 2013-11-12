options.floating = true;
robot = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
rand('twister', 1253);

nq = getNumDOF(robot);

base_name = 'utorso';
base_index = robot.findLinkInd(base_name);

end_effector_name = 'l_hand+l_hand_point_mass';
end_effector_index = robot.findLinkInd(end_effector_name);

[~, joint_path, ~] = robot.findKinematicPath(base_index, end_effector_index);
joint_names = robot.getJointNames();

joint_indices = cell2mat({robot.getBody(joint_path).dofnum})';
nq_path = length(joint_indices);

disp([joint_names(joint_path) num2cell(joint_indices)])

point = [0.05; 0.05; 0]; % doesn't work for [0; 0; 0]
rotation_type = 0; % no rotation included

n_measurements = 50;
calibration_index = 3; % number of equations per measurement
W = zeros(n_measurements * calibration_index, nq_path);
condition_numbers = zeros(n_measurements, 1);

for i = 0 : n_measurements
    q = rand(nq, 1) * pi - pi / 2;
    kinsol = robot.doKinematics(q);
    [~,J] = robot.forwardKin(kinsol, end_effector_index, point, rotation_type);
    start_row = calibration_index * i + 1;
    W(start_row : start_row + calibration_index - 1, :) = J(:, joint_indices);
    condition_numbers(i + 1) = cond(W);
end

plot(condition_numbers);
xlabel('number of measurements');
ylabel('condition number');

