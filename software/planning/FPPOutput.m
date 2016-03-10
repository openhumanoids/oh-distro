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
      
      obj.iteration_sets = results.getElementsByTagName('iteration_set');
      
      obj.n_iteration_sets = obj.iteration_sets.getLength();
      if obj.n_iteration_sets < 1
        error('No results found');
      end
    end
    
    function varargout = getData(obj, varargin)
      
      opt.hand =  {'left'};
      opt.scene = {'scene1'};
      opt.model = {'val2'};
      opt.vars = {'computation_time'};
      
      optNames = fieldnames(opt);
      nArgs = length(varargin);
      if round(nArgs/2)~=nArgs/2
        error('Needs propertyName/propertyValue pairs')
      end
      for pair = reshape(varargin,2,[])
        inpName = lower(pair{1});
        if any(strcmp(inpName,optNames))
          opt.(inpName) = {pair{2}};
        else
          error('%s is not a recognized parameter name',inpName)
        end
      end
      if isnumeric(opt.scene{1})
        sc = opt.scene{1};
        for i = 1:length(sc)
          opt.scene{i} = sprintf('scene%d', sc(i));
        end
      end
      if strcmp(opt.hand{1}, 'both')
        opt.hand = {'left', 'right'};
      end
      if iscell(opt.vars{1})
        opt.vars = opt.vars{1};
      end
      if iscell(opt.model{1})
        opt.model = opt.model{1};
      end
      
      n_dimensions = numel(varargin) / 2 - ismember('vars', {varargin{1:2:end}});
      dimensions = zeros(n_dimensions, 1);
      for i = 1:n_dimensions
        dimensions(i) = numel(opt.(varargin{2*i - 1}));
      end
      for var = 1:numel(opt.vars)
        cell_data = cell(dimensions');
        for i = 0:obj.iteration_sets.getLength()-1
          idx = [];
          for j = 1:n_dimensions
            att = char(obj.iteration_sets.item(i).getAttribute(varargin{2*j-1}));
            new_idx = find(ismember(opt.(varargin{2*j-1}), att));
            if ~isempty(new_idx)
              idx(end+1) = new_idx;
            end
          end
          if length(idx) == n_dimensions
            new_data_format = char(obj.iteration_sets.item(i).getElementsByTagName(opt.vars{var}).item(0).getAttribute('format'));
            new_data_string = char(obj.iteration_sets.item(i).getElementsByTagName(opt.vars{var}).item(0).getFirstChild().getNodeValue());
            new_data = sscanf(new_data_string, new_data_format, obj.n_iterations);
            idx = num2cell(idx);
            cell_data{idx{:}} = [cell_data{idx{:}} new_data];
          end
        end
        data = zeros([dimensions' obj.n_iterations]);
        for i = 1:numel(cell_data)
          sub = cell(1,n_dimensions);
          [sub{:}] = ind2sub(n_dimensions, i);
          data(sub{:},:) = cell_data{sub{:}};
        end
        varargout{var} = data;
      end
%       for j = 1:numel(attribute_map)
% %         sub = cell(1,n_dimensions);
% %         [sub{:}] = ind2sub(dimensions', j);
%         attribute_map{j} = [mod(j, numel(attribute_map)/dimensions(1)),...
%                             mod(j, numel(attribute_map)/dimensions(1)),...;
%       end
%       attributes = setdiff({varargin{1:2:end}}, 'vars');
%       varargout = {};
%       for v = 1:numel(opt.vars)
%         data = cell(dimensions);
%         for i = obj.iteration_sets
%           for a = attributes
%             
%           end
%         end
%         eval([opt.vars{v}, '=[];'])
%         eval(['varargout{end + 1} = ', opt.vars{v}, ';']);
%       end
%       if nargin < 4, model = 'val2'; end
%       if nargin < 3, hand = 'left'; end
%       if nargin < 2
%         scene = 'scene1';
%       elseif isnumeric(scene)
%         scene = ['scene', str2num(scene)];
%       end
      
    end
    
  end
  
  
%       attributes = [];
%       for i = 0:n_iteration_sets-1
%         iteration_set = iteration_sets.item(i);
%         if iteration_set.hasAttributes()
%           attribute_list = iteration_set.getAttributes();
%           for att = 0:attribute_list.getLength()-1
%             att_name = char(attribute_list.item(att).getNodeName());
%             if ~isfield(attributes, att_name)
%               attributes.(att_name) = {};
%             end
%             if ~any(ismember(attributes.(att_name), char(attribute_list.item(att).getNodeValue())))
%               attributes.(att_name) = [attributes.(att_name), char(attribute_list.item(att).getNodeValue())];
%             end
%           end
% 
%             variable = iteration_set.getFirstChild();
%             variable_names = {};
%             while ~isempty(variable)
%               if variable.getNodeType() ~= variable.ELEMENT_NODE
%                 variable = variable.getNextSibling();
%                 continue
%               else
%                 variable_name = char(variable.getNodeName());
%                 if ~any(ismember(variable_names, variable_name))
%                   variable_names = [variable_names, variable_name];
%                 end            
%                 variable = variable.getNextSibling();
%               end
%             end
% 
%         end
%       end
%     end    
%   end
  
 
  
%   if numel(variable_names) > 1
%     prompt = 'Select desired variable:\n';
%     for i = 1:numel(variable_names)
%       prompt = [prompt, sprintf('\t%d. %s\n', i, variable_names{i})];
%     end
%     selection = 0;
%     while ~isempty(selection) && ~any(1:numel(variable_names) == selection)
%       selection = input(prompt);
%     end
%     desired_variable = variable_names{selection};
%   end
%   
%   for i = fieldnames(attributes)'
%     if numel(attributes.(i{1})) < 2
%       attributes = rmfield(attributes, i);
%     end
%   end
%   
%   att_names = fieldnames(attributes);
%   if numel(att_names) > 1
%       prompt = 'Select independent variable\n';
%       for i = 1:numel(att_names)
%         prompt = [prompt, sprintf('\t%d. %s\n', i, att_names{i})];
%       end
%       selection = 0;
%       while ~isempty(selection) && ~any(1:numel(att_names) == selection)
%         selection = input(prompt);
%       end
%       independent_variable = att_names{selection};
%       att_names(selection) = [];
%   else
%     independent_variable = att_names{1};
%   end
%   
%   if numel(att_names) > 0
%       prompt = 'Select grouping variable (press enter for no grouping)\n';
%       for i = 1:numel(att_names)
%         prompt = [prompt, sprintf('\t%d. %s\n', i, att_names{i})];
%       end
%       selection = 0;
%       while ~isempty(selection) && ~any(1:numel(att_names) == selection)
%         selection = input(prompt);
%       end
%       if ~isempty(selection)
%         grouping_variable = att_names{selection};
%       else
%         grouping_variable = [];
%       end
%   end
%   
%   data_array = [];
%   for i = attributes.(independent_variable)
%     eval([i{:}, '=[];']);
%   end
%   
%   node_array = [];
%   for i = 0:n_iteration_sets-1
%     iteration_set = iteration_sets.item(i);
%     independent_variable_value = char(iteration_set.getAttribute(independent_variable));
%     second_idx = ismember(attributes.(indipendent_variable), independent_variable_value);
%     if ~isempty(grouping_variable)
%       grouping_variable_value = char(iteration_set.getAttribute(grouping_variable));
%       third_idx = ismember(attributes.(grouping_variable), independent_variable_value);
%     else
%       third_idx = 1;
%     end
%     node_array(
%     
%     variable_node = iteration_set.getElementsByTagName(desired_variable);
%     if variable_node.getLength == 1
%       variable_node = variable_node.item(0);
%       independent_variable_value = char(iteration_set.getAttribute(independent_variable));
%       independent_variable_idx = find(ismember(attributes.(independent_variable), independent_variable_value));
%       result_string = char(variable_node.getFirstChild().getNodeValue());
%       eval([independent_variable_value, '(end+1:end+n_iterations) = sscanf(result_string, char(variable_node.getAttribute(''format'')), n_iterations);']); 
%     end
%   end
%   eval(['data_array = [', sprintf('%s'' ', attributes.(independent_variable){:}), '];']);  
%   
%   figure;
%   if exist('aboxplot', 'file') == 2 && exist('colorgrad', 'file') == 2 && exist('quartile', 'file') == 2
%     aboxplot(data_array, 'OutlierMarker', '+')
%     title(sprintf('%d iterations computed on %s', n_iterations, time_string))
%     set(gca,'XTickLabel', attributes.(independent_variable));
%     ylabel(strrep(desired_variable, '_', ' '));
% %     if ~isempty(grouping_variable)
% %       legend(strrep(variables{group_var}, '_', ' '))
% %     end
%   end
end
