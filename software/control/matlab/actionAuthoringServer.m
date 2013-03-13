function actionAuthoringServer(options)

if(nargin<1)
    options.IK = true;
    options.sequence = false;
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
    % parse the action sequence
    
    q_bk = q;
    try
      ikargs={};
      if(options.sequence)
          action_sequence = ActionSequence();
      end
      for i=1:msg.num_contact_goals
        goal = msg.contact_goals(i);
        if (goal.contact_type==goal.ON_GROUND_PLANE)
          body=findLink(r,char(goal.object_1_name));
          collision_group = find(strcmpi(char(goal.object_1_contact_grp),body.collision_group_name));
          if isempty(collision_group) error('couldn''t find collision group %s on body %s',char(goal.object_1_contact_grp),char(goal.object_1_name)); end
          p=[goal.target_pt.x; goal.target_pt.y; goal.target_pt.z];
          pos.min=p-goal.target_pt_radius*[1;1;0];
          pos.max=p+goal.target_pt_radius*[1;1;0];
          if(goal.x_relation == 0)
              pos.min(1) = p(1);
              pos.max(1) = p(1);
          elseif(goal.x_relation == 1)
              pos.max(1) = min(pos.max(1),p(1));
          elseif(goal.x_relation == 2)
              pos.min(1) = max(pos.min(1),p(1));
          end
          if(goal.y_relation == 0)
              pos.min(2) = p(2);
              pos.max(2) = p(2);
          elseif(goal.y_relation == 1)
              pos.max(2) = min(pos.max(2),p(2));
          elseif(goal.y_relation == 2)
              pos.min(2) = max(pos.min(2),p(2));
          end
          if(goal.z_relation == 0)
              pos.min(3) = p(3);
              pos.max(3) = p(3);
          elseif(goal.z_relation == 1)
              pos.max(3) = min(pos.max(3),p(3));
          elseif(goal.z_relation == 2)
              pos.min(3) = max(pos.min(3),p(3));
          end
          ikargs={ikargs{:},body,collision_group,pos};
          if(options.sequence)
              tspan = [goal.lb_completion_time goal.ub_completion_time];
              action_constraint = ActionKinematicConstraint(body,collision_group,pos,tspan,body.linkname);
              action_sequence = action_sequence.addKinematicConstraint(action_constraint);
              
          end
        end
      end
      % If the action sequence is specified, we need to solve the ZMP
      % planning and IK for the whole sequence.
      if(options.sequence)
          dt = 0.01;
          window_size = floor((action_sequence.tspan(end)-action_sequence.tspan(1))/dt);
          zmp_planner = ZMPplanner(window_size,r.num_contacts,dt,9.81,struct('supportPolygonConstraints',true));
          t_breaks = action_sequence.tspan(1)+dt*(0:window_size-1);
          contact_tol = 1e-4;
          contact_pos = zeros(2,r.num_contacts, window_size);
          contact_flag = false(r.num_contacts,window_size);
          for i = 1:length(t_breaks)
              ikargs = action_sequence.getIKArguments(t_breaks(i));
              num_contacts = 0;
              for j = 3:3:length(ikargs)
                  for k = 1:size(ikargs{j}.max,2)
                      if(ikargs{j}.max(3,k)<contact_tol)
                          num_contacts = num_contact+1;
                          contact_pos(:,num_contacts,i) = mean([ikargs{j}.max(1:2,k) ikargs{j}.min(1:2,k)],2);
                          contact_flag(num_contacts,i) = true;
                      end
                  end
              end
          end
%           com_plan = zmp_planner.planning(
      end
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

