function I = regexpIndex(string, elements)
% I = regexpIndex(string, elements)
% where string is a regexp expression, or string to search for
% and elements is a cell array of strings
% apply regexp(string,elements{i}) and return the list of indices I
% for which the regexp is non-empty

  f = @(x) ~isempty(regexp(x,string, 'once'));
  I = find(cell2mat(cellfun(f,elements,'UniformOutput',false)));
end