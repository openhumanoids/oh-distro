classdef TrajectoryTrackingTestAnalysis
  properties
    plan_times
    execution_times
    plan_positions
    execution_positions
    joint_names
    robot
  end
  
  methods
    function obj = TrajectoryTrackingTestAnalysis(file_name)
      obj.robot = RigidBodyManipulator([getDrakePath(),'/../../models/val_description/urdf/valkyrie_sim_drake.urdf'], struct('floating', true));
      
      
      document = xmlread(file_name);  
      results = document.getDocumentElement();
      test_list = results.getElementsByTagName('test');
      for i = 0:test_list.getLength() - 1
        test = test_list.item(i);
        test_name = char(test_list.item(i).getAttribute('name'));
        plan_node = test.getElementsByTagName('committed_plan').item(0);
        plan_element_list = plan_node.getElementsByTagName('position');

        execution_node = test.getElementsByTagName('executed_plan').item(0);  
        execution_element_list = execution_node.getElementsByTagName('position');

        plan_time_string = char(plan_node.getElementsByTagName('time').item(0).getFirstChild().getNodeValue());
        obj.plan_times.(test_name) = sscanf(plan_time_string, '%d') / 1e6;
        execution_time_string = char(execution_node.getElementsByTagName('time').item(0).getFirstChild().getNodeValue());
        execution_time = double(sscanf(execution_time_string, '%g')) / 1e6;
        execution_time = (execution_time - execution_time(1));

        execution_time_start = find(execution_time >= execution_time(end)-obj.plan_times.(test_name)(end), 1);
        obj.execution_times.(test_name) = execution_time(execution_time_start:end) - execution_time(execution_time_start);
        
        for joint = 0:plan_element_list.getLength()-1
          joint_name = plan_element_list.item(joint).getAttribute('joint_name');
          if numel(obj.joint_names) < joint + 1
            obj.joint_names{joint + 1} = char(joint_name);
          end
          plan_position_string = char(plan_element_list.item(joint).getFirstChild().getNodeValue());
          obj.plan_positions.(test_name).(char(joint_name)) = sscanf(plan_position_string, '%g');
          for j = 0:execution_element_list.getLength()-1
            if strcmp(execution_element_list.item(j).getAttribute('joint_name'), joint_name)
              execution_position_string = char(execution_element_list.item(j).getFirstChild().getNodeValue());
              execution_position = sscanf(execution_position_string, '%g');
              obj.execution_positions.(test_name).(char(joint_name)) = execution_position(execution_time_start:end);
              break;
            end
          end
        end
      end
    end
    
    function plotJointTrajectories(obj, test_name, joints)
      if nargin < 3, joints = obj.joint_names; end
      if ischar(joints), joints = {joints}; end
      close all;
      for joint = 1:numel(joints)
        joint_name = joints{joint};
        figure();
        plot(obj.plan_times.(test_name), obj.plan_positions.(test_name).(joint_name), 'b')
        hold on
        plot(obj.execution_times.(test_name), obj.execution_positions.(test_name).(joint_name), 'r')
        title(joint_name)
      end
    end
    
    function plotEndEffectorTrajectory(obj, end_effector_name, test_name)
      end_effector = obj.robot.findLinkId(end_effector_name);
      plan_length =  numel(obj.plan_times.(test_name));
      plan_poses = zeros(6, plan_length);
      execution_length =  numel(obj.execution_times.(test_name));
      execution_poses = zeros(6, execution_length);
      q = zeros(obj.robot.num_positions, 1);
      for i = 1:plan_length
        for b = obj.robot.body(3:end)
        	q(b.position_num) = obj.plan_positions.(test_name).(b.jointname)(i);
        end
        kinsol = obj.robot.doKinematics(q);
        plan_poses(:,i) = obj.robot.forwardKin(kinsol, end_effector, [0;0;0], 1);
      end
      for i = 1:execution_length
        for b = obj.robot.body(3:end)
        	q(b.position_num) = obj.execution_positions.(test_name).(b.jointname)(i);
        end
        kinsol = obj.robot.doKinematics(q);
        execution_poses(:,i) = obj.robot.forwardKin(kinsol, end_effector, [0;0;0], 1);
      end
      labels = {'x position','y position','z position', 'roll', 'pitch', 'yaw'};
      close all;
      for i = 1:6
        subplot(2, 3, i);
        plot(obj.plan_times.(test_name), plan_poses(i,:), 'b');
        hold on
        plot(obj.execution_times.(test_name), execution_poses(i,:), 'r');
        legend('plan', 'execution')
        title(labels{i});
      end
    end
    
  end
end