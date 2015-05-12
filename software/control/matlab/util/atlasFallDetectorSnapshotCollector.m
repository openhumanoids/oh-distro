function atlasFallDetectorDataSnapshotCollector(output_directory)
%NOTEST

if nargin < 1
  output_directory = [getenv('DRC_BASE'),'/software/control/matlab/data/fallDetectorData/'];
end

% gui
f = figure('Position',[360,500,400,200]);
% Construct the components.
hfalling    = uicontrol('Style','pushbutton',...
             'String','Falling','Position',[65,50,70,25],...
             'Callback',{@button_Callback, true});
hnotfalling    = uicontrol('Style','pushbutton',...
             'String','Not Falling','Position',[145,50,70,25],...
             'Callback',{@button_Callback, false});
hname    = uicontrol('Style','edit',...
             'String','default','Position',[50,125,300,25]);
align([hname],'Center','None');
f.Visible = 'on';

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
qp_controller_input_monitor = drake.util.MessageMonitor(drake.lcmt_qp_controller_input,'timestamp');
lc.subscribe('QP_CONTROLLER_INPUT', qp_controller_input_monitor);

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'fallDetectorLogger');

foot_indices_struct.l_foot_fz_idx = find(strcmp('l_foot_fz',force_torque_frame.coordinates));
foot_indices_struct.l_foot_tx_idx = find(strcmp('l_foot_tx',force_torque_frame.coordinates));
foot_indices_struct.l_foot_ty_idx = find(strcmp('l_foot_ty',force_torque_frame.coordinates));
foot_indices_struct.r_foot_fz_idx = find(strcmp('r_foot_fz',force_torque_frame.coordinates));
foot_indices_struct.r_foot_tx_idx = find(strcmp('r_foot_tx',force_torque_frame.coordinates));
foot_indices_struct.r_foot_ty_idx = find(strcmp('r_foot_ty',force_torque_frame.coordinates));

foot_indices_struct.rfoot_ind = r.findLinkId('r_foot');
foot_indices_struct.lfoot_ind = r.findLinkId('l_foot');

nq = getNumPositions(r);

while true
  drawnow;
end

function inPoly = button_Callback(source,callbackdata, falling)
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    % get estimated state
    q = x(1:nq);
    qd = x(nq+(1:nq));
    kinsol = doKinematics(r,q);
    [com,J] = getCOM(r,kinsol);
    
    fc = getMessage(contact_est_monitor);
    if ~isempty(fc)
      fcmsg = drc.foot_contact_estimate_t(fc);
      % for drawing:
      left_foot_in_contact = fcmsg.left_contact > 0.5;
      right_foot_in_contact = fcmsg.right_contact > 0.5;
      cpos = struct('right', terrainContactPositions(r,kinsol,foot_indices_struct.rfoot_ind),...
                    'left', terrainContactPositions(r,kinsol,foot_indices_struct.lfoot_ind));
      if ~left_foot_in_contact
        cpos.left = mean(cpos.left, 2); % only count the center of the foot, not all the way out to its edges
      end
      if ~right_foot_in_contact
        cpos.right = mean(cpos.right, 2); % only count the center of the foot, not all the way out to its edges
      end
    else
      warning('dont have est foot contact');
    end    
    % cpos = terrainContactPositions(r,kinsol,[foot_indices_struct.rfoot_ind, foot_indices_struct.lfoot_ind]); 

    qp_controller_input_msg_data = getMessage(qp_controller_input_monitor);
    if ~isempty(qp_controller_input_msg_data)
      qp_controller_input_msg = drake.lcmt_qp_controller_input(qp_controller_input_msg_data);
    else
      warning('dont have qp controller input');
    end

    if (~isempty(fc)) && ~isempty(qp_controller_input_msg_data)
      force_torque = getMessage(force_torque_frame);

      % log er out. Save the serialized qpcontrollerinputmsgdata because
      % we can't save the message itself
      save_snapshot(t,x,force_torque,fc,qp_controller_input_msg_data,falling);

      % draw
      cop = getMeasuredCOP(r,force_torque,kinsol,foot_indices_struct);
         
      icp = getInstantaneousCapturePoint(r,com,J,qd);
      icp = [icp;min([cpos.right(3, :) cpos.left(3, :)])]; % ground projection

      lcmgl.glColor3f(color(1), color(2), color(3));
      lcmgl.sphere(icp, 0.03, 20, 20);
      lcmgl.switchBuffers();

    end
  else
    warning('no robot state!');
  end
end

function inPoly = save_snapshot(t, x, force_torque, fc, qp_controller_input_msg_data, falling)
  filename = get(hname,'String');
  if (isempty(filename))
    warning('no filename');
  else
    uniquetimestamp = datestr(now, '_yyyy_MM_dd_HH.mm.ss');
    if (falling)
      filename = [output_directory 'falling_' filename uniquetimestamp '.mat'];
    else
      filename = [output_directory 'notfalling_' filename uniquetimestamp '.mat'];
    end
    fprintf('Saving to '); disp(filename)
    save(filename,'t','x', 'force_torque', 'fc', 'qp_controller_input_msg_data', 'falling');
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

end
