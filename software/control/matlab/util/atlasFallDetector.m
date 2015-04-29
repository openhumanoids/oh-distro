function atlasFallDetector(publish_lcmgl)
%NOTEST

if nargin < 1
  publish_lcmgl = false;
end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')
options.visual = false; % loads faster
options.floating = true;
options.ignore_friction = true;
options.atlas_version = 5;

r = DRCAtlas([],options);
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);

% setup frames
state_frame = r.getStateFrame();
state_frame.subscribe('EST_ROBOT_STATE');
force_torque_frame = drcFrames.AtlasForceTorque();
force_torque_frame.subscribe('EST_ROBOT_STATE');

contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
lc = lcm.lcm.LCM.getSingleton();
lc.subscribe('FOOT_CONTACT_ESTIMATE',contact_est_monitor);

controller_status_monitor = drake.util.MessageMonitor(drc.controller_status_t, 'utime');
lc.subscribe('CONTROLLER_STATUS', controller_status_monitor);

foot_indices_struct.l_foot_fz_idx = find(strcmp('l_foot_fz',force_torque_frame.coordinates));
foot_indices_struct.l_foot_tx_idx = find(strcmp('l_foot_tx',force_torque_frame.coordinates));
foot_indices_struct.l_foot_ty_idx = find(strcmp('l_foot_ty',force_torque_frame.coordinates));
foot_indices_struct.r_foot_fz_idx = find(strcmp('r_foot_fz',force_torque_frame.coordinates));
foot_indices_struct.r_foot_tx_idx = find(strcmp('r_foot_tx',force_torque_frame.coordinates));
foot_indices_struct.r_foot_ty_idx = find(strcmp('r_foot_ty',force_torque_frame.coordinates));

foot_indices_struct.rfoot_ind = r.findLinkId('r_foot');
foot_indices_struct.lfoot_ind = r.findLinkId('l_foot');

nq = getNumPositions(r);

if publish_lcmgl
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'fall-detector');
end

debounce_threshold = 0.01; % seconds
icp_exit_time = -1; % seconds

falling_msg = drc.utime_t();
while true
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    % get estimated state
    q = x(1:nq);
    qd = x(nq+(1:nq));
    kinsol = doKinematics(r,q);
    [com,J] = getCOM(r,kinsol);
    
    fc = getMessage(contact_est_monitor);
    if ~isempty(fc)
      msg = drc.foot_contact_estimate_t(fc);
      left_foot_in_contact = msg.left_contact > 0.5;
      right_foot_in_contact = msg.right_contact > 0.5;
      cpos = struct('right', terrainContactPositions(r,kinsol,foot_indices_struct.rfoot_ind),...
                    'left', terrainContactPositions(r,kinsol,foot_indices_struct.lfoot_ind));
      if ~left_foot_in_contact
        cpos.left = mean(cpos.left, 2); % only count the center of the foot, not all the way out to its edges
      end
      if ~right_foot_in_contact
        cpos.right = mean(cpos.right, 2); % only count the center of the foot, not all the way out to its edges
      end
    else
      cpos = terrainContactPositions(r,kinsol,[foot_indices_struct.rfoot_ind, foot_indices_struct.lfoot_ind]); 
    end    
    % cpos = terrainContactPositions(r,kinsol,[foot_indices_struct.rfoot_ind, foot_indices_struct.lfoot_ind]); 

    controller_status_msg_data = getMessage(controller_status_monitor);
    if ~isempty(controller_status_msg_data)
      controller_status_msg = drc.controller_status_t(controller_status_msg_data);
      if controller_status_msg.state ~= controller_status_msg.DUMMY
        force_torque = getMessage(force_torque_frame);  
        cop = getMeasuredCOP(r,force_torque,kinsol,foot_indices_struct);
          
        icp = getInstantaneousCapturePoint(r,com,J,qd);
        icp = [icp;min(cpos(3,:))]; % ground projection

        icpIsOK = inSupportPolygon(r,icp,cpos);
        
        msg = drc.atlas_fall_detector_status_t();
        msg.utime = t*10e6;
        msg.icp = icp;
        msg.measured_cop = cop;
        msg.falling=false;
        if icpIsOK
          color = [0 1 0];
          icp_exit_time = -1;
        else
          if icp_exit_time == -1
            icp_exit_time = t;
          else
            if t-icp_exit_time > debounce_threshold
              color = [1 0 0];
              msg.falling=true;
              falling_msg.utime = msg.utime;
              % lc.publish('BRACE_FOR_FALL',falling_msg);
            end
          end
        end
        lc.publish('ATLAS_FALL_STATE',msg);
        if msg.falling
          lc.publish('RECOVERY_TRIGGER_ON', drc.utime_t());
        end

        if publish_lcmgl
          lcmgl.glColor3f(color(1), color(2), color(3));
          lcmgl.sphere(icp, 0.03, 20, 20);
          lcmgl.switchBuffers();
        end
      end
    end
  end
end

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
