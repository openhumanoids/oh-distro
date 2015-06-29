function parCapabilityMap(options)
  
  if nargin < 1 || isempty(options), options = struct(); end
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  if ~isfield(options,'visualize'), options.visualize = false; end;
  if ~isfield(options,'robot'), options.robot = []; end;  
  
  if ~isfield(options,'diameter'), options.diameter = 0.05; end; 
  if ~isfield(options,'nPointsPerSphere'), options.nPointsPerSphere = 50; end;
  if ~isfield(options,'nSamples'), options.nSamples = 100000; end;
  if ~isfield(options,'nOrient'), options.nOrient = 5; end;
  if ~isfield(options,'angTolerance'), options.angTolerance = 1*pi/180; end;
  if ~isfield(options,'posTolerance'), options.posTolerance = 0.01; end;
  tic
  options.nSamples = ceil(options.nSamples/4);
  spmd
    [map, options] = capabilityMap(options);
  end
  toc
  map = map{1}|map{2}|map{3}|map{4};
  options = options{1};
  options.nSamples = options.nSamples * 4;
  save capabilityMap map options
  drawCapabilityMap(map, options);
end