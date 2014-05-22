classdef Biped < TimeSteppingRigidBodyManipulator
  properties
    foot_contact_offsets
    foot_bodies_idx
    lc
  end

  methods
    function obj = Biped(urdf,dt,options)
      if nargin < 3
        options = struct();
        options.floating = true;
      end
      if nargin < 2
        dt = 0.002;
      end
      obj = obj@TimeSteppingRigidBodyManipulator(urdf,dt,options);
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.foot_bodies_idx = struct('right', findLinkInd(obj, 'r_foot'), 'left', findLinkInd(obj, 'l_foot'));
      obj.foot_contact_offsets = obj.findContactOffsets();
    end

    % function [pos, width] = feetPosition(obj, q0)
    function foot_center = feetPosition(obj, q0)
      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_bodies_idx.right,obj.foot_contact_offsets.right.center,true);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_bodies_idx.left,obj.foot_contact_offsets.left.center,true);

      foot_center = struct('right', rfoot0, 'left', lfoot0);
    end
  end

  methods (Static)
    function Xo = stepCenter2FootCenter(Xc, is_right_foot, nom_step_width)
      Xo = Biped.footCenter2StepCenter(Xc, is_right_foot, -nom_step_width);
    end

    function Xc = footCenter2StepCenter( Xo, is_right_foot, nom_step_width)
      % Convert a position of the center of one of the biped's feet to the
      % corresponding point half the step width toward the bot's center.
      % nom_step_width should be scalar or vector of size(1, size(Xo,2))
      if length(nom_step_width) == 1
        nom_step_width = repmat(nom_step_width, 1, size(Xo, 2));
      end
      if is_right_foot
        offs = [zeros(1,length(nom_step_width)); -nom_step_width/2; zeros(1,length(nom_step_width))];
      else
        offs = [zeros(1,length(nom_step_width)); nom_step_width/2; zeros(1,length(nom_step_width))];
      end
      for j = 1:length(Xo(1,:))
        M = rpy2rotmat(Xo(4:6,j));
        d = M * offs(:,j);
        Xc(:,j) = [Xo(1:3,j) - d(1:3); Xo(4:end,j)];
      end
    end
  end
end
