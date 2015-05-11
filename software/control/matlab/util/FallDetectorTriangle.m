classdef FallDetectorTriangle < FallDetector
  methods
    function obj = FallDetectorTriangle(r, publish_lcmgl)
      obj = obj@FallDetector(r, publish_lcmgl);
    end

     function [falling, icp, cop] = classify(obj, t,x,force_torque,fc,qp_controller_input_msg)
      % get estimated state
      q = x(1:obj.nq);
      qd = x(obj.nq+(1:obj.nq));
      kinsol = doKinematics(obj.r,q);
      [com,J] = getCOM(obj.r,kinsol);

      % if available, get foot contacts
      if ~isempty(fc)
        msg = drc.foot_contact_estimate_t(fc);
        left_foot_in_contact = msg.left_contact > 0.5;
        right_foot_in_contact = msg.right_contact > 0.5;
        cpos = struct('right', terrainContactPositions(obj.r,kinsol,obj.foot_indices_struct.rfoot_ind),...
                      'left', terrainContactPositions(obj.r,kinsol,obj.foot_indices_struct.lfoot_ind));
        if ~left_foot_in_contact
          cpos.left = mean(cpos.left, 2); % only count the center of the foot, not all the way out to its edges
        end
        if ~right_foot_in_contact
          cpos.right = mean(cpos.right, 2); % only count the center of the foot, not all the way out to its edges
        end
        cpos = [cpos.right, cpos.left];
      else
        cpos = terrainContactPositions(obj.r,kinsol,[obj.foot_indices_struct.rfoot_ind, obj.foot_indices_struct.lfoot_ind]); 
      end
      cop = obj.getMeasuredCOP(obj.r,force_torque,kinsol,obj.foot_indices_struct);
        
      icp = obj.getInstantaneousCapturePoint(obj.r,com,J,qd);
      icp = [icp;min(cpos(3,:))]; % ground projection

      if (size(cpos, 2) > 2)
        icpIsOK = obj.inSupportPolygon(obj.r,icp,cpos);
      else
        icpIsOK = false;
      end
      falling = ~icpIsOK;
    end
  end
end