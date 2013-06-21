function [result] = findFall(filename)

result.filename = filename;
[~,name,~] = fileparts(result.filename);
result.name = name;
result.vars = csv_to_mat(result.filename);

result.stats = getFallSegments(result.vars)

result.stats = error_stats(result.vars);

plotstates(result.stats);

