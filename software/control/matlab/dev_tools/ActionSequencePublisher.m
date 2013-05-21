classdef ActionSequencePublisher
	properties
		channel;
    lc;
    msg;
    robot;
    robot_state_coder;
    x0_nom;
	end

	methods
		function obj = ActionSequencePublisher(channel,kincons,x0,msg)
      if nargin < 3
        load('data/aa_atlas_fp.mat');
        x0 = xstar;
      end
      if nargin < 4
        orig_num_contact_goals = 0;
      else
        orig_num_contact_goals = msg.num_contact_goals;
      end
      obj.lc = lcm.lcm.LCM.getSingleton();
			obj.channel = channel;
      if isempty(kincons)
        obj.robot = RigidBodyManipulator('../../models/mit_gazebo_models/mit_robot_drake/model_simple_visuals_minimal_contact_point_hands.urdf',struct('floating',true));
      else
        obj.robot = kincons(1).robot;
      end
      joint_names = obj.robot.getStateFrame.coordinates(1:getNumDOF(obj.robot));
      obj.robot_state_coder = ...
        JLCMCoder(RobotStateCoder(obj.robot.name{1}, joint_names));
      obj.msg = drc.action_sequence_t();
      obj.msg.robot_name = obj.robot.name{1};
      obj.msg.num_contact_goals = orig_num_contact_goals + length(kincons);
      obj = allocate(obj);
      for i = 1:orig_num_contact_goals
        obj.msg.contact_goals(i) = msg.contact_goals(i);
      end
      if ~isempty(kincons)
        for i = 1:numel(kincons)
          obj.msg.contact_goals(orig_num_contact_goals+i) = kincon2goal(kincons(i));
        end
      end
      if nargin > 3 && isempty(x0)
        msg.q0.robot_name = 'atlas'; % To match robot_state_coder.lcmcoder
        x0 = obj.robot_state_coder.decode(msg.q0);
      end
      obj.x0_nom = x0;
		end

    function obj = allocate(obj);
      obj.msg.contact_goals = ...
        javaArray('drc.contact_goal_t',obj.msg.num_contact_goals);
    end

		function publish(obj, ik_time, x0)
      if nargin < 3
        x0 = obj.x0_nom;
      end
      obj.msg.ik_time = ik_time;
      obj.msg.q0 = obj.robot_state_coder.encode(ik_time,x0);

      obj.lc.publish(obj.channel, obj.msg);
		end

	end

end
