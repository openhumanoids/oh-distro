function replaySim(scene, iteration)

if nargin > 0
    if nargin < 2, iteration = 1; end

    warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
    warning('off', 'MATLAB:class:mustReturnObject')

    path= fileparts(which('replaySim'));
    if isdir([path ,'/', scene])    
        cd([path, '/', 'scene1'])
        try
            load(sprintf('%03d.mat',iteration), 'simVars')
        catch
            fprintf('Cannot find iteration %d of %s', iteration, scene)
        end
    else
        fprintf('Cannot find %s folder', scene)
    end
else 
    simVars = evalin('base', 'simVars');
    statVars = evalin('base', 'statVars');
end


%Unpack simVars
names = fieldnames(simVars);
for i=1:length(names)
n = names{i};
eval([n '= simVars.(names{i});'])
end

options = statVars.options;

w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'planning_mode'), options.planning_mode = 'rrt_connect'; end;
if ~isfield(options,'visualize'), options.visualize = true; end;
if ~isfield(options,'scene'), options.scene = 1; end;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;
urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
r = RigidBodyManipulator(urdf,options);

world = r.findLinkId('world');

r = Scenes.generate(options, r, world);
r = r.compile();
warning(w);

v = r.constructVisualizer();

TA = TA.setLCMGL('TA',[1,0,0]);
TB = TB.setLCMGL('TB',[0,0,1]);
T_smooth = T_smooth.setLCMGL('T_smooth', [1,0,0]);
TConnected = TConnected.setLCMGL('TConnected', [1,0,1]);

drawTree(TA);
drawTree(TB);
drawPath(T_smooth, path_ids_A);
drawPath(TConnected, path_ids_C);

q_path = extractPath(T_smooth, path_ids_A);
path_length = size(q_path,2);

q_idx = TA.idx{TA.cspace_idx};

% Scale timing to obey joint velocity limits
% Create initial spline
q_traj = PPTrajectory(pchip(linspace(0, 1, path_length), q_path(q_idx,:)));
t = linspace(0, 1, 10*path_length);
q_path = eval(q_traj, t);

% Determine max joint velocity at midpoint of  each segment
t_mid = mean([t(2:end); t(1:end-1)],1);
v_mid = max(abs(q_traj.fnder().eval(t_mid)), [], 1);

% Adjust durations to keep velocity below max
t_scaled = [0, cumsum(diff(t).*v_mid/mean(options.joint_v_max))];
tf = t_scaled(end);

% Warp time to give gradual acceleration/deceleration
t_scaled = tf*(-real(acos(2*t_scaled/tf-1)+pi)/2);
[t_scaled, idx_unique] = unique(t_scaled,'stable');

xtraj = PPTrajectory(pchip(t_scaled,[q_path(:,idx_unique); zeros(r.getNumVelocities(),numel(t_scaled))]));
xtraj = xtraj.setOutputFrame(r.getStateFrame());
key = [];
while isempty(key)
    fprintf('Playing %s iteration %d\n', scene, iteration)
    v.playback(xtraj);
    key = input('Press return to replay animation or eneter any other key to exit', 's');
end

warning('on', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('on', 'MATLAB:class:mustReturnObject')
end
