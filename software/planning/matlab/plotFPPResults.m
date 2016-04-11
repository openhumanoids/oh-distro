function plotFPPResults(input_file)
  
%   This function plots resutls obtained from batchFPP. The desired
%   variable is plotted against an independent variable that is specified
%   in an intercative prompt. A second independent variable might be
%   specifid to group boxes in the plot.
%   @param input - results struct or path to result struct
  
  if ischar(input_file)
    input_file = load(input_file);
%     fnames = fieldnames(input_file);
%     input_file = input_file.(fnames{1});
  else
    error('Unknown input type')
  end
  
  
  var_names = {'model'; 'scene'; 'hand'; 'variable'};
  group_var = false(numel(var_names), 1);
  ind_var = false(numel(var_names), 1);
  Variables = {fieldnames(input_file)};
  current_cell = input_file;
  for var = 1:numel(var_names)
    fnames = fieldnames(current_cell.(Variables{var}{1}));
    if var == 1
      fnames = fnames(~strcmp(fnames, 'details'));
    end
    current_cell = current_cell.(Variables{var}{1});
    if numel(fnames) > 1
      prompt = sprintf('%1$s SELECTION (press enter to use it as independent variable)\n', upper(var_names{var}));
      for i = 1:numel(fnames)
        prompt = [prompt, sprintf('\t%d. %s\n', i, fnames{i})];
      end
      prompt = [prompt, sprintf('\t%d. Use %s as grouping variable\n', numel(fnames)+1, var_names{var})];
      selection = 0;
      while ~isempty(selection) && ~any(1:numel(fnames)+1 == selection)
        selection = input(prompt);
      end
      if isempty(selection)
        Variables{var+1} = fnames;
        ind_var(var) = true;
      elseif selection >  numel(fnames)
        Variables{var+1} = fnames;
        group_var(var) = true;
      else
        Variables{var+1} = fnames(selection);
      end
    else
      Variables{var+1} = fnames(1);
    end
  end
 
  variables = Variables(2:end);
  fnames = fieldnames(input_file);
  input_file = input_file.(fnames{1});
  
  if any(group_var)
    data = NaN(numel(variables{group_var}), numel([input_file.(variables{1}{1}).(variables{2}{1}).(variables{3}{1}).(variables{4}{1})]), ...
               numel(variables{ind_var}));
    for i = 1:numel(variables{ind_var})
      for j = 1:numel(variables{group_var})
        vect = [input_file.(variables{1}{max(1,i*ind_var(1)+j*group_var(1))}). ...
                      (variables{2}{max(1,i*ind_var(2)+j*group_var(2))}). ...
                      (variables{3}{max(1,i*ind_var(3)+j*group_var(3))}). ...
                      (variables{4}{max(1,i*ind_var(4)+j*group_var(4))})]';
        if isnumeric(vect) || islogical(vect)
          data(j,:, i) = vect;
        end
      end
    end
  else
    data = NaN(numel([input_file.(variables{1}{1}).(variables{2}{1}).(variables{3}{1}).(variables{4}{1})]), ...
               numel(variables{ind_var}));
    for i = 1:numel(variables{ind_var})
      vect = [input_file.(variables{1}{max(1,i*ind_var(1))}). ...
                    (variables{2}{max(1,i*ind_var(2))}). ...
                    (variables{3}{max(1,i*ind_var(3))}). ...
                    (variables{4}{max(1,i*ind_var(4))})]';
      if (isnumeric(vect) || islogical(vect)) && ~isempty(vect)
        data(:,i) = vect;
      else
        error('%s data not found or not in a plottable format', variables{4}{max(1,i*ind_var(4))})
      end
    end
  end
  
  figure;
  if exist('aboxplot', 'file') == 2 && exist('colorgrad', 'file') == 2 && exist('quartile', 'file') == 2
    aboxplot(data, 'OutlierMarker', '+')
    title(sprintf('%d iterations computed on %s', input_file.details.n_iterations, input_file.details.time))
    set(gca,'XTickLabel',variables{ind_var});
    ylabelstr = strrep(variables{4}{1}, '_', ' ');
    for v = 2:numel(variables{4})
      ylabelstr = [ylabelstr, ', ', strrep(variables{4}{v}, '_', ' ')];
    end
    ylabel(ylabelstr)
    if any(group_var)
      legend(strrep(variables{group_var}, '_', ' '))
    end
  end
end