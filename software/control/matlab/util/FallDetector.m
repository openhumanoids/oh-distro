classdef FallDetector
  properties
    r;
    nq;
    lcmgl;
    publish_lcmgl;

    % should redo with runLCM
    lc;
    state_frame;
    force_torque_frame;
    contact_est_monitor;
    foot_indices_struct;
    controller_status_monitor;
    qp_controller_input_monitor;

    icp_exit_time = -1.0;
  end

  properties (Constant)
    DEBOUNCE_THRESHOLD = 0.01; % seconds
  end

  methods
    function obj = FallDetector(r, publish_lcmgl)
      obj.r = r;
      obj.state_frame = r.getStateFrame();
      obj.state_frame.subscribe('EST_ROBOT_STATE');
      obj.force_torque_frame = drcFrames.AtlasForceTorque();
      obj.force_torque_frame.subscribe('EST_ROBOT_STATE');

      obj.publish_lcmgl = publish_lcmgl;
      if (publish_lcmgl)
        obj.lcmgl = LCMGLClient('fall_detector');
      end

      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);

      obj.controller_status_monitor = drake.util.MessageMonitor(drc.controller_status_t, 'utime');
      obj.lc.subscribe('CONTROLLER_STATUS', obj.controller_status_monitor);

      obj.qp_controller_input_monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input,'timestamp');
      obj.lc.subscribe('QP_CONTROLLER_INPUT', obj.qp_controller_input_monitor);

      obj.foot_indices_struct.l_foot_fz_idx = find(strcmp('l_foot_fz',obj.force_torque_frame.coordinates));
      obj.foot_indices_struct.l_foot_tx_idx = find(strcmp('l_foot_tx',obj.force_torque_frame.coordinates));
      obj.foot_indices_struct.l_foot_ty_idx = find(strcmp('l_foot_ty',obj.force_torque_frame.coordinates));
      obj.foot_indices_struct.r_foot_fz_idx = find(strcmp('r_foot_fz',obj.force_torque_frame.coordinates));
      obj.foot_indices_struct.r_foot_tx_idx = find(strcmp('r_foot_tx',obj.force_torque_frame.coordinates));
      obj.foot_indices_struct.r_foot_ty_idx = find(strcmp('r_foot_ty',obj.force_torque_frame.coordinates));
      obj.foot_indices_struct.rfoot_ind = obj.r.findLinkId('r_foot');
      obj.foot_indices_struct.lfoot_ind = obj.r.findLinkId('l_foot');

      obj.nq = getNumPositions(obj.r);
    end

    function run(obj)
      falling_msg = drc.utime_t();
      while true
        [x,t] = getNextMessage(obj.state_frame,1);
        if ~isempty(x)
          fc = getMessage(obj.contact_est_monitor);
          force_torque = getMessage(obj.force_torque_frame);  
          controller_status_msg_data = getMessage(obj.controller_status_monitor);
          qp_controller_input_msg_data = getMessage(obj.qp_controller_input_monitor);
          if ~isempty(controller_status_msg_data) && ~isempty(qp_controller_input_msg_data)
            controller_status_msg = drc.controller_status_t(controller_status_msg_data);
            qp_controller_input_msg = drake.lcmt_qp_controller_input(qp_controller_input_msg_data);
            if controller_status_msg.state ~= controller_status_msg.DUMMY
              [falling, icp, cop] = obj.classify(t, x, force_torque, fc, qp_controller_input_msg);

              falling_debounced = false;
              if ~falling
                color = [0 1 0];
                obj.icp_exit_time = -1;
              else
                if obj.icp_exit_time == -1
                  obj.icp_exit_time = t;
                else
                  if t-obj.icp_exit_time > obj.DEBOUNCE_THRESHOLD
                    color = [1 0 0];
                    falling_debounced=true;
                  end
                end
              end
              msg = drc.fall_detector_status_t();
              msg.utime = t*10e6;
              msg.icp = icp;
              msg.measured_cop = cop;
              msg.falling=falling_debounced;
              obj.lc.publish('FALL_STATE',msg);
              if msg.falling
                recovery_msg = drc.recovery_trigger_t();
                recovery_msg.activate = true;
                recovery_msg.override = false;
                obj.lc.publish('RECOVERY_TRIGGER', recovery_msg);
              end
              if obj.publish_lcmgl
                obj.lcmgl.glColor3f(color(1), color(2), color(3));
                obj.lcmgl.sphere(icp, 0.03, 20, 20);
                obj.lcmgl.switchBuffers();
              end
            end
          end
        end
      end
    end
  end


  methods(Abstract)
    classify(obj, t,x,force_torque,fc,qp_controller_input_msg)
  end

  methods(Static)
    function cop=getMeasuredCOP(r,force_torque,kinsol,foot_indices_struct)
      fz_l = force_torque(foot_indices_struct.l_foot_fz_idx);
      tx_l = force_torque(foot_indices_struct.l_foot_tx_idx);
      ty_l = force_torque(foot_indices_struct.l_foot_ty_idx);
      l_foot_pt = [-ty_l/fz_l; tx_l/fz_l; 0];

      fz_r = force_torque(foot_indices_struct.r_foot_fz_idx);
      tx_r = force_torque(foot_indices_struct.r_foot_tx_idx);
      ty_r = force_torque(foot_indices_struct.r_foot_ty_idx);
      r_foot_pt = [-ty_r/fz_r; tx_r/fz_r; 0];

      lfoot_pos = forwardKin(r,kinsol, foot_indices_struct.lfoot_ind, l_foot_pt);
      rfoot_pos = forwardKin(r,kinsol, foot_indices_struct.rfoot_ind, r_foot_pt);

      cop = (fz_l*lfoot_pos + fz_r*rfoot_pos)/(fz_l+fz_r);
      cop(3) = cop(3)-0.0811;
    end


    function icp=getInstantaneousCapturePoint(r,com,J,qd)
      icp = com(1:2) + 1/sqrt(9.81/com(3))*J(1:2,:)*qd;
    end


    function inPoly = inSupportPolygon(r,pt,cpos)
      convh = convhull(cpos(1,:), cpos(2,:));
      inPoly = inpolygon(pt(1), pt(2), cpos(1,convh), cpos(2,convh));
    end
  end
end


