function actionAuthoringIKServer

% listens for drc_action_sequence_t messages and, upon receipt, computes the
% IK and publishes the robot_state_t

lc = lcm.lcm.LCM.getSingleton(); %('udpm://239.255.76.67:7667?ttl=1');

% construct lcm input monitor
monitor = drake.util.MessageMonitor(drc.action_sequence_t(),'utime');
lc.subscribe('action_authoring_plan_action_request',monitor);

% construct lcm state publisher
% todo: should really load model name from lcm
r = RigidBodyManipulator('drake/examples/Atlas/urdf/atlas_minimal_contact.urdf',struct('floating',true));
joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
joint_names = regexprep(joint_names, 'pelvis', 'base', 'preservecase'); % change 'pelvis' to 'base'
robot_state_coder = LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(),'x',JLCMCoder(RobotStateCoder('atlas', joint_names)));

% load the "zero position"
load('data/atlas_fp3.mat');
q0 = xstar(1:getNumDOF(r));

timeout=10;
while (1)
  data = getNextMessage(monitor,timeout);
  if ~isempty(data)
    msg = drc.action_sequence_t(data);
    % parse the action sequence
    
    ikargs={};
    for i=1:msg.num_contact_goals
      goal = msg.contact_goals(i);
      if (goal.contact_type==goal.ON_GROUND_PLANE)
%        body=findLink(r,char(goal.object_1_name));
        body=findLink(r,'r_foot');
        pos=goal.ground_plane_pt;
        ikargs={ikargs{:},body,[pos.x;pos.y;pos.z]};
      end
    end
    
    % call IK 
    q = inverseKin(r,q0,ikargs{:});
    
    % publish robot state message
    x = [q;0*q];
    publish(robot_state_coder,0,x,'ACTION_AUTHORING_IK_ROBOT_STATE');
  end
end

