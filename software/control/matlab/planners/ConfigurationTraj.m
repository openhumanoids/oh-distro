classdef ConfigurationTraj
  properties
    qtraj;
  end

  methods
    function obj = ConfigurationTraj(qtraj_pp)
      obj.qtraj = qtraj_pp; % ppform
    end

    function msg = toLCM(obj)
      msg=drc.configuration_traj_t();
      msg.utime = 0;
      msg.qtraj = mxSerialize(obj.qtraj);
      msg.n_qtraj_bytes = length(msg.qtraj);
    end
  end
end
