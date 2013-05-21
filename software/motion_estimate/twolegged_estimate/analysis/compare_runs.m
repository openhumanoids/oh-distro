function results = compare_runs(topdir, do_plots)

d = dir(topdir);
files = {};
for i = 1:numel(d)
    if (d(i).isdir)
        filename = [topdir,'/',d(i).name,'/true_estimated_states.csv'];
        if (exist(filename,'file'))
            files{end+1} = filename;
        end
    end
end

results = struct('filename',{},'name',{},'vars',{},'stats',{});
for i = 1:numel(files)
    result.filename = files{i};
    [~,name,~] = fileparts(fileparts(result.filename));
    result.name = name;
    result.vars = csv_to_mat(result.filename);
    result.stats = error_stats(result.vars);
    results(i) = result;
    fprintf('computed %d/%d, %s\n', i, numel(files), result.name);
end

if (exist('do_plots','var') && do_plots)
    stats = [results.stats];
    vels = [stats.vel];
    names = {results.name};
    
    figure(10);
    clf;
    title('rms error');
    ylabel('rms error');
    vals = cat(1,vels.rms_error);
    plot(vals);
    grid on;
    xticklabel_rotate(1:numel(names), 90, names, 'interpreter','none');

    figure(11);
    clf;
    title('max error');
    ylabel('max error');
    vals = cat(1,vels.max_error);
    plot(vals);
    grid on;
    xticklabel_rotate(1:numel(names), 90, names, 'interpreter','none');

    figure(12);
    clf;
    title('num exceeding');
    ylabel('num exceeding');
    vals = cat(1,vels.pct_exceeding);
    plot(vals);
    grid on;
    xticklabel_rotate(1:numel(names), 90, names, 'interpreter','none');
end
