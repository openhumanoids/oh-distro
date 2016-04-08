classdef FPPOutput
  
  properties
    n_iterations
    time_string
    scenes
    grasping_hands
    models
    iteration_sets
    n_iteration_sets
  end
  
  methods
    
    function obj = FPPOutput(file_name)
      if nargin < 1, file_name = 'output.fpp'; end

      document = xmlread(file_name);
      results = document.getDocumentElement();
      details_node = results.getElementsByTagName('details');
      if details_node.getLength() ~= 1
        error('This file is not correctly formatted');
      else
        details_node = details_node.item(0);
      end
  
      obj.n_iterations = sscanf(char(details_node.getElementsByTagName('n_iterations').item(0).getFirstChild().getNodeValue()), '%d');
      obj.time_string = char(details_node.getElementsByTagName('time').item(0).getFirstChild().getNodeValue());
      obj.scenes = strsplit(char(details_node.getElementsByTagName('scenes').item(0).getFirstChild().getNodeValue()));
      obj.grasping_hands = strsplit(char(details_node.getElementsByTagName('grasping_hands').item(0).getFirstChild().getNodeValue()));
      obj.models = strsplit(char(details_node.getElementsByTagName('models').item(0).getFirstChild().getNodeValue()));
      
      iteration_set_list = results.getElementsByTagName('iteration_set');
      for i = 0:iteration_set_list.getLength() - 1
        obj.iteration_sets{i+1} = iteration_set_list.item(i);
      end
      
      obj.n_iteration_sets = numel(obj.iteration_sets);
      if obj.n_iteration_sets < 1
        error('No results found');
      end
    end
    
    function output = getData(obj, varargin)
      
      [opt, n_dimensions, dimensions, variables] = obj.parseInput(varargin{:});
      
      cell_data = cell([dimensions(1:end-1) 1]);
      for v = 1:numel(opt.vars)
        for i = 1:obj.n_iteration_sets
          idx = [];
          for j = 1:numel(variables)
            if ~strcmp(variables{j}, 'vars')
              att = char(obj.iteration_sets{i}.getAttribute(variables{j}));
              new_idx = find(ismember(opt.(variables{j}), att));
            else
              new_idx = v;
            end
            if ~isempty(new_idx)
              idx(j) = new_idx;
            end
          end
          if ~any(idx == 0)
            new_data_format = char(obj.iteration_sets{i}.getElementsByTagName(opt.vars{v}).item(0).getAttribute('format'));
            new_data_string = char(obj.iteration_sets{i}.getElementsByTagName(opt.vars{v}).item(0).getFirstChild().getNodeValue());
            new_data = sscanf(new_data_string, new_data_format, obj.n_iterations);
            idx = num2cell(idx);
            cell_data{idx{:}} = [cell_data{idx{:}}; new_data];
          end
        end
      end
      for i = 1:numel(cell_data)
        sub = cell(1,ndims(cell_data));
        [sub{:}] = ind2sub(dimensions, i);
        if ~isempty(cell_data{sub{:}})
          output(sub{:},:) = cell_data{sub{:}};
        else
          str = '';
          for j = 1:numel(sub)
            str = [str, sprintf(' %s = %s', variables{j}, opt.(variables{j}){sub{j}})];
          end
          warning('No results found for %s', str);
        end
      end
      output = squeeze(output);
    end
    
  end
  
  methods (Access=private)
    
    function [opt, n_dimensions, dimensions, variables] = parseInput(obj, varargin)
      
      attributes = {'hand', 'scene', 'model', 'vars'};
      
      n_args = length(varargin);
      if round(n_args/2)~=n_args/2
        error('Needs propertyName/propertyValue pairs')
      elseif ~any(ismember(varargin(1:2:end), 'vars'))
        error('vars must be specified')
      end
      for pair = reshape(varargin,2,[])
        input_name = lower(pair{1});
        if any(strcmp(input_name,attributes))
          opt.(input_name) = pair{2};
          switch input_name
            case 'hand'
              if strcmp(opt.(input_name), 'both')
                opt.(input_name) = {'left', 'right'};
              end
            case 'scene'
              if isnumeric(opt.(input_name))
                sc = opt.scene;
                opt.scene = {};
                for i = 1:length(sc)
                  opt.scene{i} = sprintf('scene%d', sc(i));
                end
              end
          end
          if ~iscell(opt.(input_name))
            opt.(input_name) = {opt.(input_name)};
          end
        else
          error('%s is not a recognized parameter name',input_name)
        end
      end
      
      n_dimensions = numel(fieldnames(opt)) + 1;
      variables = fieldnames(opt);
      var_count = zeros(1, n_dimensions - 1);
      filtered_iteration_sets = obj.iteration_sets;
      for f = 1:numel(variables)
        dimensions(f) = numel(opt.(variables{f}));
        if ~strcmp('vars', variables{f})
          for i = numel(filtered_iteration_sets):-1:1
            if ~strcmp(char(filtered_iteration_sets{i}.getAttribute(variables{f})), opt.(variables{f}){1})
              filtered_iteration_sets(i) = [];
            end
          end
          if numel(filtered_iteration_sets) == 0
            error('No results found for %s = %s', variables{f}, opt.(variables{f}){1})
          end
        end
      end
      dimensions(end+1) = numel(filtered_iteration_sets) * obj.n_iterations;
    end
  end
end
