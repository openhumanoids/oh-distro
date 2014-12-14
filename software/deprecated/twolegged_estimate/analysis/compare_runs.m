function results = compare_runs(csv_dir, do_plots)

d = dir([csv_dir,'/*.csv']);
files = {};
for i = 1:numel(d)
    filename = [csv_dir,'/',d(i).name];
    files{end+1} = filename;
end

% sort by A,B,C
filelist = cell(size(files));
for i = 1:numel(files)
    [~,b,c] = fileparts(files{i});
    filelist{i} = [b,c];
end
a_matches = ~cellfun(@isempty,strfind(filelist,'_A_'));
b_matches = ~cellfun(@isempty,strfind(filelist,'_B_'));
c_matches = ~cellfun(@isempty,strfind(filelist,'_C_'));
files = [sort(files(a_matches)),sort(files(b_matches)),sort(files(c_matches))];

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
