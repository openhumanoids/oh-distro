function plotResults(varargin)
  
  opt = struct('files', dir('*.mat'), 'print', false, 'variables', []);
  optNames = fieldnames(opt);
  nArgs = length(varargin);
  if round(nArgs/2)~=nArgs/2
    error('Needs propertyName/propertyValue pairs')
  end
  for pair = reshape(varargin,2,[])
    inpName = lower(pair{1});
    if any(strcmp(inpName,optNames))
      opt.(inpName) = pair{2};
    else
      error('%s is not a recognized parameter name',inpName)
    end
  end
  
  warning('off', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
  warning('off', 'MATLAB:class:mustReturnObject')
  
  n = 0;
  
  files = opt.files;
  for i = length(files):-1:1
    vars = whos('-file', files(i).name);
    if length(vars)~= 3 || ~all(strcmp({'Infos', 'SimVars', 'StatVars'}, {vars.name}))
      files(i) = [];
    else
      if n > 0 
        if vars(1).size(2) ~= n
          error('number of iterations mismatch')
        end
      else
         n = vars(1).size(2);
      end
    end
  end
  
  if ~isempty(files)
    nScenes = length(files);
    load(files(1).name);
    percSuccess = zeros(nScenes, 1);
    StatVars = [StatVars{[Infos.status] == Info.SUCCESS}];
    if isempty(opt.variables)
      variables = fieldnames(StatVars);
    elseif ischar(opt.variables)
      variables = {opt.variables};
    else
      variables = opt.variables;
      for v = variables'
        if ~ismember(char(v), fieldnames(StatVars))
          error('Results for %s not found', char(v))
        end
      end
    end
    numericVariables = variables;
    for i = length(variables):-1:1
      if ~isnumeric([StatVars.(variables{i})]);
        numericVariables(i) = [];
      end
    end
    for v = variables'
      eval([char(v) '= {};']);
    end
    
    for i = 1:length(files)
      load(files(i).name);      
      StatVars = [StatVars{[Infos.status] == Info.SUCCESS}];
%       scene = StatVars(1).options.scene;
      percSuccess(i) = nnz([Infos.status] == Info.SUCCESS)/n*100;
      if percSuccess(i) > 0
        for v = variables'
          eval([v{:} '(i)= {[StatVars(:).' v{:} ']''};']);
        end
      end
    end
  else
    error('No result files found');
  end
  
  close all
  clf;
  nw = ceil(sqrt(length(numericVariables)+1));
  nh = ceil((length(numericVariables)+1)/nw);
  labels = arrayfun(@(i) sprintf('Scene %d', i), 1:nScenes, 'UniformOutput', false);
  
  if isempty(opt.variables) || ismember('info', variables)
    [~, axNum] = ismember('info', variables');
    subplot(nh, nw, axNum);
    bar(percSuccess);
    set(gca, 'xticklabel', labels);
    ylabel('successful iterations [%]');
    if opt.print
      fig = figure('visible', 'off');
      bar(percSuccess);
      set(gca, 'xticklabel', labels);
      ylabel('successful iterations [%]');
      print('-depsc', 'successfulIterations.eps')
      close(fig);
    end
  end
  
  if exist('aboxplot', 'file') == 2 && exist('colorgrad', 'file') == 2 && exist('quartile', 'file') == 2
    for i = numericVariables'
      if ~strcmp(char(i), 'options')
        [~, axNum] = ismember(char(i), variables');
        subplot(nh, nw, axNum);
        aboxplot(eval(char(i)), 'OutlierMarker', '+');
        set(gca,'XTick', 1-0.7/2+(2*(1:nScenes)-1)*0.7/(2*nnz(percSuccess > 0)));
        set(gca,'XTickLabel',labels);
        ylabel(char(i))
        if opt.print
          fig = figure('visible', 'off');
          aboxplot(eval(char(i)), 'OutlierMarker', '+');
          set(gca,'XTick', 1-0.7/2+(2*(1:nScenes)-1)*0.7/(2*nnz(percSuccess > 0)));
          set(gca,'XTickLabel',labels);
          ylabel(char(i))
          print('-depsc', [char(i) '.eps'])
          close(fig);
        end
      end
    end
  else
    error('Advanced Box Plot not Found');
  end
  
  warning('on', 'Drake:RigidBodyManipulator:UnsupportedContactPoints')
  warning('on', 'MATLAB:class:mustReturnObject')
  
end