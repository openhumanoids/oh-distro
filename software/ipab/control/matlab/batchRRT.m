function batchRRT(n, varargin)
% batchRRT(n, varargin)
% 
% This function runs a set of n iteration of exploringRRT.m for a
% particular set of scenes and saves results to be plotted with
% plotResults.m
% 
% @param n          Number of iteration for each scene
% @param varargin   name/value pairs of options. Possible values are:
%                   'path': folder path to  save results [string]
%                   (default: current directory)
%                   'scenes': the scenes to include [intger array]
%                   (default(all scenes defined in class Scenes)
%                   'visualize': visualize the resulting motion for each
%                   iteration [bool] (default: false)
%                   'savesimvars': save the structure simvars containing
%                   variables to replay the resulting motion [bool]
%                   (default: false)
  
  options = struct('path', pwd, 'scenes', 1:Scenes.getnScenes(),...
              'visualize', false, 'savesimvars', false, 'planning_mode', 'multiRRT',...
              'model', 'val2', 'graspingHand', 'right');
  optNames = fieldnames(options);
  nArgs = length(varargin);
  if round(nArgs/2)~=nArgs/2
    error('Needs propertyName/propertyValue pairs')
  end
  for pair = reshape(varargin,2,[])
    inpName = pair{1};
    if any(strcmp(inpName,optNames))
      options.(inpName) = pair{2};
    else
      error('%s is not a recognized parameter name',inpName)
    end
  end
  
  mkdir(options.path)
  cd(options.path)
  warning('off', 'MATLAB:MKDIR:DirectoryExists');
  warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
  warning('off', 'MATLAB:Java:ConvertFromOpaque')
  for scene = options.scenes
    if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
    if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
    if ~isfield(options,'convex_hull'), options.convex_hull = false; end;
    if ~isfield(options,'costType'), options.costType = 'length'; end;
    if ~isfield(options,'firstFeasibleTraj'), options.firstFeasibleTraj = false; end;


    options.floating = true;
    options.terrain = MyRigidBodyFlatTerrain(); %Changed to a smaller terrain to avoid visualization problem when zooming
    options.joint_v_max = 15*pi/180;
    
    %Set up the simulation, run it n times and save the results
    options.scene = scene;
    options.robot = Scenes.generateScene(options);
    SimVars = {};
    StatVars = {};
    Infos = {};%Info.empty(0, n);
    parfor i = 1:n
      fprintf('Starting scene %d iteration %d ...\n', scene, i)
      try
        [~, info, simVars, statVars] = exploringRRT(options);
        if options.savesimvars
          SimVars{i} = simVars;
        end
        StatVars{i} = statVars;
        Infos{i} = info;
      catch err
%         load lastRndg
%         save(sprintf('Rndg_Scene%d_Iteration%d', scene, i), 'rndSeed')
        fprintf('\tScene %d itereation %d failed with the following error:\n\t\t%s\n\tStack:\n',...
          scene, i, err.message)
        for st = 1:length(err.stack)
          fprintf('\t\t%s in line %d\n', err.stack(st).name, err.stack(st).line)
        end
%         Infos(i) = Info(Info.FAIL_OTHER);
      end
      fprintf('Finished scene %d iteration %d ...\n', scene, i)
    end
%     if options.savesimvars
%       SimVars = cell2mat(SimVars);
%     end
%     StatVars = cell2mat(StatVars);
    fprintf('Saving data ...\n')
    save(sprintf('Results_Scene%02d', scene), 'SimVars', 'StatVars', 'Infos')
    fprintf('Results_Scene%02d saved\n', scene)
  end
  
  if ~strcmp(options.path, pwd)
  % back to the parent dir
    cd ..
  end
  warning('on', 'MATLAB:MKDIR:DirectoryExists');
  warning('on', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
  warning('on', 'MATLAB:Java:ConvertFromOpaque')
end