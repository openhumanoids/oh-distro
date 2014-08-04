classdef ConfigurationTraj
  properties
    qtraj
    link_constraints
  end

  methods
    function obj = ConfigurationTraj(qtraj_pp,link_constraints)
      obj.qtraj = qtraj_pp; % ppform
      obj.link_constraints = link_constraints; % link_constraints struct
    end

    function msg = toLCM(obj)
      msg=drc.configuration_traj_t();
      msg.utime = 0;
      msg.qtraj = mxSerialize(obj.qtraj);
      msg.n_qtraj_bytes = length(msg.qtraj);
      msg.link_constraints = mxSerialize(obj.link_constraints);
      msg.n_link_constraints_bytes = length(msg.link_constraints);
    end
  end
end
