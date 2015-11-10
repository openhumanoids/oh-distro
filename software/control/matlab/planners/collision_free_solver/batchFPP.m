function batchFPP(n, scenes, options)

  if nargin < 3 || isempty(options), options = struct(); end

  if ~isfield(options,'models'), models = {'val2'}; else models = options.models; end;
  if ~isfield(options,'grasping_hands'), grasping_hands = {'right'}; else grasping_hands = options.grasping_hands; end;
  
  for hand_idx = 1:numel(grasping_hands)
    for model_idx = 1:numel(models)
      for scene = scenes
        
        opt.floating = true;
        opt.terrain = RigidBodyFlatTerrain();
        
        opt.scene = scene;
        opt.model = models{model_idx};
        opt.graspingHand = grasping_hands{hand_idx};
        opt.convex_hull = true;
        
        opt.robot = Scenes.generateScene(opt);
        
        parfor i = 1:n
          fprintf('Computing iteration %d of scene %d with %s model and %s hand\n', ...
                  i, scene, models{model_idx}, grasping_hands{hand_idx})
          [info, debug_vars] = exploringFPP(opt);
        end
      end
    end
  end
  
end