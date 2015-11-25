function plotFPPResults(input, model, scene, hand, variable, group)
  
%   This function plots resutls obtained from batchFPP. The desired
%   variable is plotted against an independent variable that is specified
%   by passing a cell of strings or an empty array as the respective
%   argument. If two independent variables are used, group specifies wich one
%   to use to group boxes in the plot.
%   @param input - results struct or path to result struct
%   @param model - string or cell of strings with the desired robot
%   model(s)
%   @param scene - string or cell of strings with the desired scene(s)
%   @param hand - string or cell of strings with the desired grasping hand(s)
%   @param variable - string or cell of strings with the desired variable(s)
%   @param group - variable to use to group boxes in the plot
  
  if ischar(input)
    input = load(input);
    fnames = fieldnames(input);
    input = input.(fnames{1});
  else
    error('Unknown input type')
  end
  
  if ~isempty(scene)
    if isnumeric(scene)
      scene = sprintf('scene%d', scene);
    else
      error('Scene must be an integer')
    end
  end
  
  variables = {model, scene, hand, variable};
  var_names = {'model'; 'scene'; 'hand'; 'variable'};
    
  for v = 1:numel(variables)
    if ischar(variables{v})
      variables{v} = {variables{v}};
    elseif isempty(variables{v})
      variables{v} = {};
    end
  end
  
  groups = cellfun('size', variables(:), 2) ~= 1;
  if nnz(groups) > 2
    error('Only two grouping variables are allowed')
  elseif nnz(groups) < 1
    error('At least one grouping variable must be given')
  elseif nnz(groups) == 2
    if nargin < 6 || isempty(group)
      error('group cannot be empty when using two grouping variables')
    elseif ~any(strcmp(var_names, group))
      error('group must be in {%s, %s, %s, %s}', var_names{:})
    end
  elseif nnz(groups) == 1
    if nargin > 5
      warning('group is not necessary when using only onr grouping variable')
    end
  end
  
  if nargin > 5
    group_var = strcmp(var_names, group);
  else
    group_var = false(4,1);
  end
  ind_var = groups ~= group_var;
  if nnz(ind_var) > 1
    error('The group variable must have at least two fields or must be empty')
  end
  
  if isempty(variables{1})
    variables{1} = fieldnames(input);
    variables{1} = {variables{1}{~strcmp(variables{1}, 'details')}}';
  end
  if isempty(variables{2}), variables{2} = fieldnames(input.(variables{1}{1}));end
  if isempty(variables{3}), variables{3} = fieldnames(input.(variables{1}{1}).(variables{2}{1}));end
  if isempty(variables{4})
    variables{4} = fieldnames(input.(variables{1}{1}).(variables{2}{1}).(variables{3}{1}));
    variables{4} = {variables{4}{~strcmp(variables{4}, 'nullspace_iter')}}';
  end
  
  if nargin > 5
    data = NaN(numel(variables{group_var}), numel([input.(variables{1}{1}).(variables{2}{1}).(variables{3}{1}).(variables{4}{1})]), ...
               numel(variables{ind_var}));
    for i = 1:numel(variables{ind_var})
      for j = 1:numel(variables{group_var})
        vect = [input.(variables{1}{max(1,i*ind_var(1)+j*group_var(1))}). ...
                      (variables{2}{max(1,i*ind_var(2)+j*group_var(2))}). ...
                      (variables{3}{max(1,i*ind_var(3)+j*group_var(3))}). ...
                      (variables{4}{max(1,i*ind_var(4)+j*group_var(4))})]';
        if isnumeric(vect) || islogical(vect)
          data(j,:, i) = vect;
        end
      end
    end
  else
    data = NaN(numel([input.(variables{1}{1}).(variables{2}{1}).(variables{3}{1}).(variables{4}{1})]), ...
               numel(variables{ind_var}));
    for i = 1:numel(variables{ind_var})
      vect = [input.(variables{1}{max(1,i*ind_var(1))}). ...
                    (variables{2}{max(1,i*ind_var(2))}). ...
                    (variables{3}{max(1,i*ind_var(3))}). ...
                    (variables{4}{max(1,i*ind_var(4))})]';
      if isnumeric(vect) || islogical(vect)
        data(:,i) = vect;
      end
    end
  end
  
  if exist('aboxplot', 'file') == 2 && exist('colorgrad', 'file') == 2 && exist('quartile', 'file') == 2
    aboxplot(data, 'OutlierMarker', '+')
    title(sprintf('%d iterations computed on %s', input.details.n_iterations, input.details.time))
    set(gca,'XTickLabel',variables{ind_var});
    ylabelstr = strrep(variables{4}{1}, '_', ' ');
    for v = 2:numel(variables{4})
      ylabelstr = [ylabelstr, ', ', strrep(variables{4}{v}, '_', ' ')];
    end
    ylabel(ylabelstr)
    if nargin > 5
      legend(strrep(variables{group_var}, '_', ' '))
    end
  end
end