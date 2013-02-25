function actionAuthoringIKServer

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
r = RigidBodyManipulator('../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', struct('floating','true'));
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
      for i=1:msg.num_contact_goals
        goal = msg.contact_goals(i);
        if (goal.contact_type==goal.ON_GROUND_PLANE)
          body=findLink(r,char(goal.object_1_name));
          collision_group = find(strcmpi(char(goal.object_1_contact_grp),body.collision_group_name));
          if isempty(collision_group) error('couldn''t find collision group %s on body %s',char(goal.object_1_contact_grp),char(goal.object_1_name)); end
          p=[goal.ground_plane_pt.x; goal.ground_plane_pt.y; goal.ground_plane_pt.z];
          pos.min=p-goal.ground_plane_pt_radius*[1;1;0];
          pos.max=p+goal.ground_plane_pt_radius*[1;1;0];
          ikargs={ikargs{:},body,collision_group,pos};
        end
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

