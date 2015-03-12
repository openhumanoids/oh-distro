classdef ConfigurationTrajPlan < QPWalkingPlan
  properties
    link_constraints
    robot;
    V;
  end

  methods
    function obj = ConfigurationTraj(biped,qtraj_pp,link_constraints)
      typecheck(biped, 'Biped');
      obj = obj@QPWalkingPlan(biped);
      obj.qtraj = PPTrajectory(qtraj_pp); % ppform
      obj.link_constraints = link_constraints; % link_constraints struct
      [~, obj.V, obj.comtraj, 
    end

    function msg = toLCM(obj)
      msg=drc.configuration_traj_t();
      msg.utime = 0;
      msg.qtraj = mxSerialize(obj.qtraj);
      msg.n_qtraj_bytes = length(msg.qtraj);
      msg.link_constraints = mxSerialize(obj.link_constraints);
      msg.n_link_constraints_bytes = length(msg.link_constraints);
    end

    function qp_input = getQPControllerInput(obj, t_global, x, rpc)
      t_plan = t_global - obj.start_time;
      T = obj.duration;
      t_plan = min([max([t_plan, 0]), T]);

      q_des = ppval(obj.qtraj, t_plan);

      kinsol = doKinematics(obj.robot, q_des);
      com = obj.robot.getCOM(kinsol);
      feet = [obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.right, [0;0;0], 0),...
              obj.robot.forwardKin(kinsol, obj.robot.foot_frame_id.left, [0;0;0], 0)];

      qp_input = obj.default_qp_input;
      qp_input.zmp_data.D = -(com(3) - mean(feet(3,:)))/9.81 * eye(2);

    end
  end

  methods (Static)
    function obj = from_configuration_traj_t(msg)
      obj = ConfigurationTraj(mxDeserialize(msg.qtraj),...
                              mxDeserialize(msg.link_constraints));
    end
  end

end
