function results = compare_runs(csv_dir, do_plots)

d = dir([csv_dir,'/*.csv']);
files = {};
for i = 1:numel(d)
    filename = [csv_dir,'/',d(i).name];
    files{end+1} = filename;
end

results = struct('filename',{},'name',{},'vars',{},'stats',{});
for i = 1:numel(files)
    result.filename = files{i};
    [~,name,~] = fileparts(result.filename);
    result.name = name;
    result.vars = csv_to_mat(result.filename);
    result.stats = error_stats(result.vars);
    results(i) = result;
    fprintf('computed %d/%d, %s\n', i, numel(files), result.name);
end

if (do_plots)
    do_stats_plots(results);
end
