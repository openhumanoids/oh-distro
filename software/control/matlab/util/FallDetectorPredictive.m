classdef FallDetectorPredictive < FallDetector
  methods
    function obj = FallDetectorPredictive(r, publish_lcmgl)
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
        cpos_struct = struct('right', terrainContactPositions(obj.r,kinsol,obj.foot_indices_struct.rfoot_ind),...
                      'left', terrainContactPositions(obj.r,kinsol,obj.foot_indices_struct.lfoot_ind));
        cpos_contact_only = [];
        if (left_foot_in_contact)
          cpos_contact_only = [cpos_contact_only cpos_struct.left];
        end
        if (right_foot_in_contact)
          cpos_contact_only = [cpos_contact_only cpos_struct.right];
        end
        cpos = [cpos_struct.left cpos_struct.right];
      else
        cpos = terrainContactPositions(obj.r,kinsol,[obj.foot_indices_struct.rfoot_ind, obj.foot_indices_struct.lfoot_ind]); 
      end    
      cop = obj.getMeasuredCOP(obj.r,force_torque,kinsol,obj.foot_indices_struct);
        
      icp = obj.getInstantaneousCapturePoint(obj.r,com,J,qd);
      icp = [icp;min(cpos(3,:))]; % ground projection

      if (size(cpos_contact_only, 2) > 2)
        icpIsOK = obj.inSupportPolygon(obj.r,icp,cpos_contact_only);
      else
        icpIsOK = false;
      end
      if (~icpIsOK)
        % Second chance: will we be caught in our plan?

        forward_projection_time = 0.0;
        if ~left_foot_in_contact
          % take the farthest-in-time target pose we have for this
           % foot.
          this_bmd = 0;
          for i=1:length(qp_controller_input_msg.body_motion_data)
            body_or_frame_id = qp_controller_input_msg.body_motion_data(i).body_id;
            if (body_or_frame_id < 0)
              body_or_frame_id = obj.r.getFrame(body_or_frame_id).body_ind;
            end
            if strcmp(obj.r.getBodyOrFrameName(body_or_frame_id), 'l_foot')
              this_bmd = qp_controller_input_msg.body_motion_data(i);
              break;
            end
          end
          if (this_bmd == 0)
            cpos_struct.left = mean(cpos_struct.left, 2); % only count the center of the foot, not all the way out to its edges
          else
            forward_projection_time = 0;
            for j=1:this_bmd.spline.num_segments
              end_time = this_bmd.spline.breaks(j+1) - this_bmd.spline.breaks(j);
              end_pt = zeros(6, 1);
              for i=1:6
                coefs = this_bmd.spline.polynomial_matrices(j).polynomials(i,1).coefficients;
                end_pt(i) = end_time^3*coefs(4)+end_time^2*coefs(3)+end_time^1*coefs(2)+coefs(1);
              end
              if (size(cpos_struct.left, 2) == 1 && norm(cpos_struct.left - end_pt(1:3))<1E-3)
                break;
              else
                cpos_struct.left = end_pt(1:3);
                forward_projection_time = this_bmd.spline.breaks(j+1) - t;
              end
            end
          end
          if (obj.publish_lcmgl)
            obj.lcmgl.glColor3f(0, 0, 1);
            obj.lcmgl.sphere(cpos_struct.left, 0.03, 20, 20);
          end
        end

        if ~right_foot_in_contact
           % take the farthest-in-time target pose we have for this
           % foot.
          this_bmd = 0;
          for i=1:length(qp_controller_input_msg.body_motion_data)
            body_or_frame_id = qp_controller_input_msg.body_motion_data(i).body_id;
            if (body_or_frame_id < 0)
              body_or_frame_id = obj.r.getFrame(body_or_frame_id).body_ind;
            end
            if strcmp(obj.r.getBodyOrFrameName(body_or_frame_id), 'r_foot')
              this_bmd = qp_controller_input_msg.body_motion_data(i);
              break;
            end
          end
          if (this_bmd == 0)
            cpos_struct.right = mean(cpos_struct.right, 2); % only count the center of the foot, not all the way out to its edges
          else
            forward_projection_time = 0;
            for j=1:this_bmd.spline.num_segments
              end_time = this_bmd.spline.breaks(j+1) - this_bmd.spline.breaks(j);
              end_pt = zeros(6, 1);
              for i=1:6
                coefs = this_bmd.spline.polynomial_matrices(j).polynomials(i,1).coefficients;
                end_pt(i) = end_time^3*coefs(4)+end_time^2*coefs(3)+end_time^1*coefs(2)+coefs(1);
              end
              if (size(cpos_struct.right, 2) == 1 && norm(cpos_struct.right - end_pt(1:3))<1E-3)
                break;
              else
                forward_projection_time = this_bmd.spline.breaks(j+1) - t; 
                cpos_struct.right = end_pt(1:3);
              end
            end
          end
          if (obj.publish_lcmgl)  
            obj.lcmgl.glColor3f(0, 0, 1);
            obj.lcmgl.sphere(cpos_struct.right, 0.03, 20, 20);
          end
        end
        % roll icp ofrward in time -- if we will be captured, we're also happy
        omega = sqrt(1/(-qp_controller_input_msg.zmp_data.D(1, 1)));
        cop = [qp_controller_input_msg.zmp_data.y0; 0];
        icp_new = obj.icpUpdate(icp, cop, forward_projection_time, omega);
        cpos_pred =[cpos_struct.left cpos_struct.right]; 
        if (size(cpos_pred, 2) > 2)
          icpIsOK = obj.inSupportPolygon(obj.r,icp_new ,cpos_pred);
        else
          icpIsOK = false;
        end
      end

      falling = ~icpIsOK;
    end
  end

 methods(Static)
    function icp_new = icpUpdate(icp, cop, dt, omega)
       icp_new = (icp - cop) * exp(dt*omega) + cop;
    end
  end
end