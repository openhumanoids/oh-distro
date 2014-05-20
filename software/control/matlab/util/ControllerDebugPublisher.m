classdef ControllerDebugPublisher

	properties
		lc;
		channel;
  end

  methods
		function obj = ControllerDebugPublisher(channel)
			obj.channel = channel;
			obj.lc = lcm.lcm.LCM.getSingleton();
    end
    
		function publish(obj, data)
      msg = drc.controller_debug_t();
      msg.utime = data.utime;
      % QP solution vector
      msg.n_alpha = length(data.alpha);
      msg.alpha = data.alpha;
      % torque input vector
      msg.n_u = length(data.u);
      msg.u = data.u;
      % active support body indices
      msg.n_active_supports = length(data.active_supports);
      msg.active_supports = data.active_supports;
      % solver return status
      msg.info = data.info;
      % qdd desired input to QP
      msg.n_q = length(data.qddot_des);
      msg.qddot_des = data.qddot_des;
      % centroidal momentum
      msg.h = data.h;
      % desired centroidal momentum rate of change
      msg.hdot_des = data.hdot_des;
      % active set of inequality constraints
      msg.n_active_constraints = length(data.active_constraints);
      msg.active_constraints = data.active_constraints;
      % redundant fields for foot contact, useful for plotting in scope
      msg.l_foot_contact = data.l_foot_contact;
      msg.r_foot_contact = data.r_foot_contact;
      % desired body acceleration inputs
      msg.n_body_acc_des = length(data.body_acc_des);
      msg.body_acc_des = data.body_acc_des;
      % QP bounds
      msg.nb = length(data.lb);
      msg.lb = data.lb;
      msg.ub = data.ub;

      obj.lc.publish(obj.channel, msg);
    end
  end

end
