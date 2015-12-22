function output = batchFPP(n, scenes, options)

  if nargin < 3 || isempty(options), options = struct(); end

  if ~isfield(options,'models')
    models = {'val2'};
  elseif ischar(options.models)
    models = {options.models};
  else
    models = options.models;
  end
  
  if ~isfield(options,'grasping_hands')
    grasping_hands = {'right'};
  elseif ~iscell(options.grasping_hands)
    grasping_hands = {options.grasping_hands};
  else
    grasping_hands = options.grasping_hands;
  end
  
  output.details.models = models;
  output.details.n_iterations = n;
  output.details.scenes = scenes;
  output.details.grasping_hands = grasping_hands;
  output.details.time = datestr(now);
  
  for model_idx = 1:numel(models)
    opt.model = models{model_idx};
    opt.convex_hull = true;
    opt.visualize = false;
    opt.verbose = false;
    opt.floating = true;
    opt.robot = Scenes.generateRobot(opt);
    for scene = scenes
      opt.scene = scene;
      for hand_idx = 1:numel(grasping_hands)
        opt.graspingHand = grasping_hands{hand_idx};
        parfor i = 1:n
            fprintf('Computing iteration %d of scene %d with %s model and %s hand\n', ...
                    i, scene, models{model_idx}, grasping_hands{hand_idx})
            [~, debug_vars] = exploringFPP(opt);
            debug_vars_array(i) = debug_vars;
        end
        output.(models{model_idx}).(sprintf('scene%d', scene)).(grasping_hands{hand_idx}) = debug_vars_array;
      end
    end
  end
  
end