function I = getStringIndices(list, elements)
if iscell(elements)
  f = @(x) find(strcmp(list,x));
  I = cell2mat(cellfun(f,elements,'UniformOutput',false))';
else
  I = find(strcmp(list,elements));
end
end