function output = batchFPP(n, scenes, file_name, options)

  if nargin < 4 || isempty(options), options = struct(); end
  if nargin < 3 || isempty(file_name), file_name = 'output.fpp'; end

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
  
  document = com.mathworks.xml.XMLUtils.createDocument('results');
  result_node = document.getDocumentElement();
  
  details_node = document.createElement('details');
  result_node.appendChild(details_node);
  
  models_node = document.createElement('models');
  models_text = document.createTextNode('');
  models_node.appendChild(models_text);
  for m = 1:numel(models)
    string = char(models_text.getNodeValue);
    if ~isempty(string)
      models_text.setNodeValue([string, ' ']);
    end
    models_text.setNodeValue([char(models_text.getNodeValue), models{m}])
  end
  details_node.appendChild(models_node);
  
  iterations_node = document.createElement('n_iterations');
  iterations_node.appendChild(document.createTextNode(sprintf('%d',n)));
  details_node.appendChild(iterations_node);
  
  scenes_node = document.createElement('scenes');
  scene_text = document.createTextNode('');
  scenes_node.appendChild(scene_text);
  for s = scenes
    string = char(scene_text.getNodeValue);
    if ~isempty(string)
      scene_text.setNodeValue([string, ' ']);
    end
    scene_text.setNodeValue([char(scene_text.getNodeValue), sprintf('scene%d', s)])
  end
  details_node.appendChild(scenes_node);
  
  hands_node = document.createElement('grasping_hands');
  hands_text = document.createTextNode('');
  hands_node.appendChild(hands_text);
  for h = 1:numel(grasping_hands)
    string = char(hands_text.getNodeValue);
    if ~isempty(string)
      hands_text.setNodeValue([string, ' ']);
    end
    hands_text.setNodeValue([char(hands_text.getNodeValue), grasping_hands{h}])
  end
  details_node.appendChild(hands_node);
  
  time_node = document.createElement('time');
  time_node.appendChild(document.createTextNode(datestr(now)));
  details_node.appendChild(time_node);
  
  f_id = fopen('matlabCapabilityMap.log','w');
  fclose(f_id);
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
            [~, debug_vars] = exploringFPP(opt, i);
            debug_vars_array(i) = debug_vars;
        end
        output.(models{model_idx}).(sprintf('scene%d', scene)).(grasping_hands{hand_idx}) = debug_vars_array;
      end
    end
  end
  for m = fieldnames(output)'
    for s = fieldnames(output.(m{1}))'
      for h = fieldnames(output.(m{1}).(s{1}))'
        iteration_set_node = document.createElement('iteration_set');
        result_node.appendChild(iteration_set_node);
        iteration_set_node.setAttribute('model', m{1});
        iteration_set_node.setAttribute('scene', s{1});
        iteration_set_node.setAttribute('hand', h{1});
        for v = fieldnames(output.(m{1}).(s{1}).(h{1}))'
          if ~strcmp(v{1}, 'formats')
            variable_node = document.createElement(v{1});
            variable_text = document.createTextNode('');
            variable_node.appendChild(variable_text);
            form = output.(m{1}).(s{1}).(h{1})(1).formats.(v{1});
            variable_node.setAttribute('format', form);
            for val = output.(m{1}).(s{1}).(h{1})
              string = char(variable_text.getNodeValue);
              if ~isempty(string)
                variable_text.setNodeValue([string, ' ']);
              end
              variable_text.setNodeValue([char(variable_text.getNodeValue), sprintf(form, val.(v{1}))])
            end
            iteration_set_node.appendChild(variable_node);
          end
        end
      end
    end
  end
    
  xmlwrite(file_name, document);
  
end