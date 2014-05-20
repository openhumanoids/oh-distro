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
    function foot_orig = feetPosition(obj, q0)
      typecheck(q0,'numeric');
      sizecheck(q0,[obj.getNumDOF,1]);

      kinsol = doKinematics(obj,q0);

      rfoot0 = forwardKin(obj,kinsol,obj.foot_bodies_idx.right,[0;0;0],true);
      lfoot0 = forwardKin(obj,kinsol,obj.foot_bodies_idx.left,[0;0;0],true);

      foot_orig = struct('right', rfoot0, 'left', lfoot0);
    end

   %  function options = getReachabilityOptions(obj, options)
   %    if ~isfield(options, 'forward_step')
   %      options.forward_step = obj.max_forward_step;
   %    end
   %    if ~isfield(options, 'nom_step_width')
   %      options.nom_step_width = obj.nom_step_width;
   %    end
   %    if ~isfield(options, 'max_step_width')
   %      options.max_step_width = max([obj.max_step_width, options.nom_step_width + 0.05]);
   %    end
   %    if ~isfield(options, 'min_step_width')
   %      options.min_step_width = min([obj.min_step_width, options.nom_step_width - 0.05]);
   %    end
   %    if ~isfield(options, 'backward_step')
   %      options.backward_step = options.forward_step;
   %    end
   %    if ~isfield(options, 'max_step_rot')
   %      options.max_outward_step_rot = obj.max_step_rot;
   %    end
   %    if ~isfield(options, 'max_inward_step_rot')
   %      options.max_inward_step_rot = 0.001; % BDI walker doesn't allow toe to rotate inward at all
   %    end
   %    if ~isfield(options, 'max_step_dz')
   %      options.max_step_dz = obj.max_step_dz;
   %    end
   %  end

   %  function [A, b] = getFootstepDiamondCons(obj, p0_is_right_foot, options)
   %    % Alternative (experimental) formulation of footstep linear constraints, in which the reachable region is defined by a diamond which extends out to the max forward and backward distances only at the nominal step width, rather than a rectangle defined by the max forward/back step and the max/min step width.

   %    if nargin < 3
   %      options = struct();
   %    end

   %    options = getReachabilityOptions(obj, options);
   %    options.forward_step = max([options.forward_step, 0.005]);
   %    options.backward_step = max([options.backward_step, 0.005]);
   %    options.max_step_width = max([options.max_step_width, options.nom_step_width + 0.005]);
   %    options.min_step_width = min([options.min_step_width, options.nom_step_width - 0.005]);
   %    options.nom_upward_step = max(options.nom_upward_step, 0.01);
   %    options.nom_downward_step = max(options.nom_downward_step, 0.01);

   %    [Axy, bxy] = poly2lincon([0, options.forward_step, 0, -options.backward_step], [options.min_step_width, options.nom_step_width, options.max_step_width, options.nom_step_width]);
   %    [Axz, bxz] = poly2lincon([0, options.nom_forward_step, options.max_forward_step, options.nom_forward_step, 0, -options.backward_step], [options.nom_upward_step, options.nom_upward_step, 0, -options.nom_downward_step, -options.nom_downward_step, 0]);
   %    A = [Axy, zeros(4, 4);
   %         Axz(:,1), zeros(size(Axz, 1), 1), Axz(:,2), zeros(size(Axz, 1), 3);
   %         0 0 1 0 0 0;
   %         0 0 -1 0 0 0;
   %         0 0 0 0 0 -1;
   %         0 0 0 0 0 1;
   %         0, 1/(options.max_step_width-options.nom_step_width), 0, 0, 0, 1/options.max_outward_step_rot;
   %         0, 1/(options.min_step_width-options.nom_step_width), 0, 0, 0, 1/options.max_outward_step_rot];
   %    if ~p0_is_right_foot
   %      A(:,2) = -A(:,2);
   %      A(:,6) = -A(:,6);
   %    end
   %    b = [bxy;
   %         bxz;
   %         options.max_step_dz;
   %         options.max_step_dz;
   %         options.max_inward_step_rot;
   %         options.max_outward_step_rot;
   %         1 + options.nom_step_width/(options.max_step_width-options.nom_step_width);
   %         1 + options.nom_step_width/(options.min_step_width-options.nom_step_width)
   %         ];
   % end

   %  function [A, b] = getFootstepLinearCons(obj, p0_is_right_foot, options)
   %    % Get the linear inequality constraints for Ax - b <= 0, where x is a column of relative step positions, as given by Biped.relativeSteps(). Automatically flips the y direction for left steps to make them equivalent to right steps.

   %    if nargin < 3
   %      options = struct();
   %    end
   %    options = getReachabilityOptions(obj, options);
   %    options.max_inward_step_rot = 0.1;

   %    A = [1 0 0 0 0 0;
   %         -1 0 0 0 0 0;
   %         0 1 0 0 0 0;
   %         0 -1 0 0 0 0;
   %         0 0 1 0 0 0;
   %         0 0 -1 0 0 0;
   %         0 0 0 0 0 -1;
   %         0 0 0 0 0 1];
   %    if ~p0_is_right_foot
   %      A(:,2) = -A(:,2);
   %      A(:,6) = -A(:,6);
   %    end
   %    b = [options.forward_step;
   %         options.backward_step;
   %         options.max_step_width;
   %         -options.min_step_width;
   %         options.max_step_dz;
   %         options.max_step_dz;
   %         options.max_inward_step_rot;
   %         options.max_outward_step_rot];
   %  end

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
