function runFromProneActionAuthoring()
s=warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
r = RigidBodyManipulator('../../../models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf', struct('floating','true'));
warning(s);
v = r.constructVisualizer();

data = load('../data/aa_atlas_fp.mat');
q = data.xstar(1:getNumDOF(r));

action_sequence_listener = ActionSequenceListener('foo bar robot','action_authoring_plan_action_request');
disp('Listening for action authoring plans...');
  waiting = true;
  action_sequence_data = [];
  while waiting
    action_sequence_data = action_sequence_listener.getNextMessage(100);
    if (~isempty(action_sequence_data))
      disp('action_sequence plan received.');
      waiting = false;
    end
  end
  action_sequence = encodeActionSequence(r,action_sequence_data);
  keyboard;
end