function atlasFallDetectorTester(data_directory)
%NOTEST

if nargin < 1
  data_directory = [getenv('DRC_BASE'),'/software/control/matlab/data/fallDetectorData/'];
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

fallDetector = FallDetectorTriangle(r, 0);

% Get all data
files = dir(data_directory);
falling_files = find(strncmp({files.name}, 'falling', 7));
falling_files = files(falling_files);
notfalling_files = find(strncmp({files.name}, 'notfalling', 10));
notfalling_files = files(notfalling_files);

tp = 0;
fp = 0;
tn = 0;
fn = 0;
targets = [];
outputs = [];
for iouter = 1:numel(falling_files)
  prediction = doTestCase([data_directory falling_files(iouter).name]);
  if (prediction ~= 1)
    fn = fn + 1;
    fprintf('FN Error on %s\n', falling_files(iouter).name);
  else
    tp = tp + 1;
  end
  targets = [targets 1];
  outputs = [outputs prediction];
end
for iouter = 1:numel(notfalling_files)
  prediction = doTestCase([data_directory notfalling_files(iouter).name]);
  if (prediction ~= 0)
    fprintf('FP Error on %s\n', notfalling_files(iouter).name);
    fp = fp + 1;
  else
    tn = tn + 1;
  end
  targets = [targets 0];
  outputs = [outputs prediction];
end
accuracy = (tp+tn)/(tp+tn+fp+fn);
targets
outputs
fprintf('tp: %d, fp: %d, tn: %d, fn: %d, acc: %f\n', tp, fp, tn, fn, accuracy);
plotroc(targets, outputs);

function prediction=doTestCase(filepath, falling_gt)
  testdata = load(filepath);
  x = testdata.x;
  t = testdata.t;
  force_torque = testdata.force_torque;
  qp_controller_input_msg_data = testdata.qp_controller_input_msg_data;
  fc = testdata.fc;
  qp_controller_input_msg = drake.lcmt_qp_controller_input(qp_controller_input_msg_data);
  
  prediction = fallDetector.classify(t,x,force_torque,fc, qp_controller_input_msg);
end

end
