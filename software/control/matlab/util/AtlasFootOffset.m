classdef AtlasFootOffset < DRCPlanner
  % Planner used to calculate the vector offset from BDI's foot pos
  % estimate to our estimate from forwardKin. Run this with
  % runAtlasFootOffset.m
  properties
    biped
  end

  methods
    function obj = AtlasFootOffset(biped)
      obj = obj@DRCPlanner();
      obj.biped = biped;
      obj = addInput(obj, 'x0', 'EST_ROBOT_STATE', obj.biped.getStateFrame().lcmcoder, 1, 1, 1);
      obj = addInput(obj, 'atlas_foot_pos', 'ATLAS_FOOT_POS_EST', 'drc.atlas_foot_pos_est_t',1,1,1);
    end
    
    function plan(obj, data)
        persistent xforms
        if isempty(xforms)
            xforms = struct('right', [], 'left', []);
        end
        q = data.x0(1:end/2);
        kinsol = doKinematics(obj.biped,q);
        fk_pos.right = forwardKin(obj.biped,kinsol,obj.biped.foot_bodies_idx(1),[0;0;0],1);
        fk_pos.left = forwardKin(obj.biped,kinsol,obj.biped.foot_bodies_idx(2),[0;0;0],1);
        at_pos.right = data.atlas_foot_pos.right_position;
        at_pos.left = data.atlas_foot_pos.left_position;
        
        R.right = inv(rpy2rotmat(fk_pos.right(4:6)));
        R.left = inv(rpy2rotmat(fk_pos.left(4:6)));
        
        xforms.right(:,end+1) = R.right * (at_pos.right - fk_pos.right(1:3));
        xforms.left(:,end+1) = R.left * (at_pos.left - fk_pos.left(1:3));
        fprintf(1, 'r: [%d, %d, %d]\n', xforms.right(:,end));
        fprintf(1, 'l: [%d, %d, %d]\n', xforms.left(:,end));
        save('foot_offset_results.mat', 'xforms');
    end
  end
end