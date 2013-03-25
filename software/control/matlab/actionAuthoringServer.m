function actionAuthoringServer(IK)
% IK = 1 means we only require an IK solution
% IK = 2 meas that the we also do ZMP planning
if(nargin<1)
    action_options.IK = true;
    action_options.ZMP = false;
elseif(IK == 1)
    action_options.IK = true;
    action_options.ZMP = false;
elseif(IK == 2)
    action_options.IK = false;
    action_options.ZMP = true;
end
% listens for drc_action_sequence_t messages and, upon receipt, computes the
% IK and publishes the robot_state_t

lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');

% construct lcm input monitor
monitor = drake.util.MessageMonitor(drc.action_sequence_t(),'utime');
lc.subscribe('action_authoring_plan_action_request',monitor);

% construct lcm state publisher
% todo: should really load model name from lcm

s=warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator('../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', struct('floating','true'));
warning(s);
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
robot_state_coder = LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(),'x',JLCMCoder(RobotStateCoder('atlas', joint_names)));
robot_plan_publisher =  RobotPlanPublisher('atlas',joint_names,true, ...
                                           'ACTION_AUTHORING_ROBOT_PLAN');
%%

% load the "zero position"
load('data/aa_atlas_fp.mat');
q = xstar(1:getNumDOF(r));

% setup IK prefs
cost = Point(r.getStateFrame,1);
cost.pelvis_x = 0;
cost.pelvis_y = 0;
cost.pelvis_z = 0;
cost.pelvis_roll = 1000;
cost.pelvis_pitch = 1000;
cost.pelvis_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
options.q_nom = q;

v = r.constructVisualizer();

timeout=10;
while (1)
  data = getNextMessage(monitor,timeout);
  if ~isempty(data)
    msg = drc.action_sequence_t(data);
    action_sequence = ActionSequence();
    if(action_options.ZMP)
        action_sequence_ZMP = ActionSequence();
    end
    q_bk = q;
    % Get initial conditions from msg.q0
    msg.q0.robot_name = 'atlas'; % To match robot_state_coder.lcmcoder
    x0 = robot_state_coder.lcmcoder.jcoder.decode(msg.q0).val;
    q = x0(1:getNumDOF(r));
    try
      for i=1:msg.num_contact_goals
        goal = msg.contact_goals(i);
        if (goal.contact_type==goal.ON_GROUND_PLANE)
          body=findLink(r,char(goal.object_1_name));
          collision_group = find(strcmpi(char(goal.object_1_contact_grp),body.collision_group_name));
          if isempty(collision_group) error('couldn''t find collision group %s on body %s',char(goal.object_1_contact_grp),char(goal.object_1_name)); end
          p=[goal.target_pt.x; goal.target_pt.y; goal.target_pt.z];
          pos = struct();
          pos.max = inf(3,1);
          pos.min = -inf(3,1);
          if(goal.x_relation == 0)
              pos.min(1) = p(1)+goal.target_pt_radius;
              pos.max(1) = p(1)-goal.target_pt_radius;
          elseif(goal.x_relation == 1)
              pos.max(1) = p(1);
              pos.min(1) = -inf;
          elseif(goal.x_relation == 2)
              pos.min(1) = p(1);
              pos.max(1) = inf;
          end
          if(goal.y_relation == 0)
              pos.min(2) = p(2)-goal.target_pt_radius;
              pos.max(2) = p(2)+goal.target_pt_radius;
          elseif(goal.y_relation == 1)
              pos.max(2) = p(2);
              pos.min(2) = -inf;
          elseif(goal.y_relation == 2)
              pos.min(2) = p(2);
              pos.max(2) = inf;
          end
%           if(goal.z_relation == 0)
%               pos.min(3) = p(3);
%               pos.max(3) = p(3);
%           elseif(goal.z_relation == 1)
%               pos.max(3) = min(pos.max(3),p(3));
%           elseif(goal.z_relation == 2)
%               pos.min(3) = max(pos.min(3),p(3));
%           end
          pos.max(3) = 0;
          pos.min(3) = 0;
          tspan = [goal.lower_bound_completion_time goal.upper_bound_completion_time];
          collision_group_pt = mean(body.getContactPoints(collision_group),2);
          action_constraint = ActionKinematicConstraint(body,collision_group_pt,pos,tspan,body.linkname);
          action_sequence = action_sequence.addKinematicConstraint(action_constraint);
          for body_ind = 1:length(r.body)
              body_contact_pts = r.body(body_ind).getContactPoints();
              if(~isempty(body_contact_pts))
                  above_ground_constraint = ActionKinematicConstraint.groundConstraint(r.body(body_ind),body_contact_pts,tspan,r.body(body_ind).linkname);
                  action_sequence = action_sequence.addKinematicConstraint(above_ground_constraint);
              end 
          end
          if(action_options.ZMP)
              % Solve the IK here to for each key time point (the starting of
              % ending of each constraint). We agree that the constraint is
              % active in the time interval [lb_completion_time,
              % ub_completion_time) (Half closed half open)
              ikargs = action_sequence.getIKArguments(tspan(1));
              if isempty(ikargs)
                q_lb_completion_time=options.q_nom;
              else
                % call IK
                q_lb_completion_time = inverseKin(r,q,ikargs{:},options);
              end
              kinsol = doKinematics(r,q_lb_completion_time);
              pos_lb_completion_time = struct();
              pos_lb_completion_time.max = forwardKin(r,kinsol,body,collision_group_pt,false);
              pos_lb_completion_time.min = forwardKin(r,kinsol,body,collision_group_pt,false);
              action_constraint_ZMP = ActionKinematicConstraint(body,collision_group_pt,pos_lb_completion_time,tspan,body.linkname);
              action_sequence_ZMP = action_sequence_ZMP.addKinematicConstraint(action_constraint_ZMP);
              for body_ind = 1:length(r.body)
                  body_contact_pts = r.body(body_ind).getContactPoints();
                  if(~isempty(body_contact_pts))
                      above_ground_constraint = ActionKinematicConstraint.groundConstraint(r.body(body_ind),body_contact_pts,tspan,r.body(body_ind).linkname);
                      action_sequence_ZMP = action_sequence_ZMP.addKinematicConstraint(above_ground_constraint);
                  end 
              end
          end
        end
      end
      % If the action sequence is specified, we need to solve the ZMP
      % planning and IK for the whole sequence.
      if(action_options.ZMP)
          dt = 0.01;
          window_size = ceil((action_sequence_ZMP.tspan(end)-action_sequence_ZMP.tspan(1))/dt);
          zmp_planner = ZMPplanner(window_size,r.num_contacts,dt,9.81);
          t_breaks = action_sequence_ZMP.tspan(1)+dt*(0:window_size-1);
          t_breaks(end) = action_sequence_ZMP.tspan(end);
          contact_tol = 1e-4;
          contact_pos = zeros(2,r.num_contacts, window_size);
          contact_flag = false(r.num_contacts,window_size);
          for i = 1:length(t_breaks)
              ikargs = action_sequence_ZMP.getIKArguments(t_breaks(i));
              num_contacts = 0;
              for j = 3:3:length(ikargs)
                  for k = 1:size(ikargs{j}.max,2)
                      if(ikargs{j}.max(3,k)<contact_tol)
                          num_contacts = num_contacts+1;
                          contact_pos(:,num_contacts,i) = mean([ikargs{j}.max(1:2,k) ikargs{j}.min(1:2,k)],2);
                          contact_flag(num_contacts,i) = true;
                      end
                  end
              end
          end
          % solve the IK for the key time samples, then interpolate the COM
          % height as the desired COM height
          q_key_time_samples = zeros(r.getNumDOF(),length(action_sequence.key_time_samples));
          com_key_time_samples = zeros(3,length(action_sequence.key_time_samples));
          for i = 1:length(action_sequence.key_time_samples)
              ikargs = action_sequence.getIKArguments(action_sequence.key_time_samples(i));
              if(isempty(ikargs))
                  q_key_time_samples(:,i) = options.q_nom;
              else
                  q_key_time_samples(:,i) = inverseKin(r,q,ikargs{:},options);
              end
              com_key_time_samples(:,i) = r.getCOM(q_key_time_samples(:,i));
          end
          % TODO: Publish constraint satisfaction message here
          com_height_traj = PPTrajectory(foh(action_sequence.key_time_samples,com_key_time_samples(3,:)));
          com_height = com_height_traj.eval(t_breaks);
          q0 = q;
          com0 = r.getCOM(q0);
          comdot0 = 0*com0;
          zmp_options = struct();
          zmp_options.supportPolygonConstraints = true;
          zmp_options.shrink_factor = 0.8;
          zmp_options.useQP = true;
          zmp_options.penalizeZMP = true;
          com_plan = zmp_planner.planning(com0(1:2),comdot0(1:2),contact_pos,contact_flag,com_height,t_breaks,zmp_options);
          q_zmp_plan = zeros(r.getNumDOF,length(t_breaks));
          q_zmp_plan(:,1) = q0;
          for i = 2:length(t_breaks)
              ikargs = action_sequence.getIKArguments(t_breaks(i));
              ikargs = [ikargs,{0},{com_plan(:,i)}];
              q_zmp_plan(:,i) = inverseKin(r,q_zmp_plan(:,i-1),ikargs{:},options);
          end

          % publish t_breaks, q_zmp_plan with RobotPlanPublisher.java
          publish(robot_plan_publisher, t_breaks, ...
                  [q_zmp_plan; zeros(size(q_zmp_plan))]);
      end
      ikargs = action_sequence.getIKArguments(action_sequence.tspan(end));
      if isempty(ikargs)
        q=options.q_nom;
      else
        % call IK
        q = inverseKin(r,q,ikargs{:},options);
      end
    catch ex
      warning(ex.identifier,ex.message);
      q=q_bk;
      continue;
    end
    % publish robot state message
    x = [q;0*q];
    v.draw(0,x);
    publish(robot_state_coder,0,x,'ACTION_AUTHORING_IK_ROBOT_STATE');
  end
end

end 
