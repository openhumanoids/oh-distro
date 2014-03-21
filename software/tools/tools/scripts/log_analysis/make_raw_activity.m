function main()
close all
d=[];
fid=fopen('log');
while 1
  tline = fgetl(fid);
  if ~ischar(tline), break, end
  disp(tline)
  str_split =textscan(tline,'%f%s',' ')
  var = str_split{2}(1);
  if ~isfield(d,var{:})
    d.(var{:}) = str_split{1}(1)
  else
    d.(var{:}) = [ d.(var{:}); str_split{1}(1) ]
  end
end
fclose(fid);

% log:
init_utime = 1387562271287314
% mission:
%init_utime = 1387562807870000

d = field_change(d,'BEHAVIOR_CHANGE_1', 'BEHAVIOR_CHANGE_FREEZE');
d = field_change(d,'BEHAVIOR_CHANGE_2', 'BEHAVIOR_CHANGE_PREP');
d = field_change(d,'BEHAVIOR_CHANGE_3', 'BEHAVIOR_CHANGE_STAND');
d = field_change(d,'BEHAVIOR_CHANGE_4', 'BEHAVIOR_CHANGE_WALK');
d = field_change(d,'BEHAVIOR_CHANGE_5', 'BEHAVIOR_CHANGE_STEP');
d = field_change(d,'BEHAVIOR_CHANGE_6', 'BEHAVIOR_CHANGE_MANIP');
d = field_change(d,'BEHAVIOR_CHANGE_7', 'BEHAVIOR_CHANGE_USER');
d = field_change(d,'BEHAVIOR_CHANGE_8', 'BEHAVIOR_CHANGE_CALIB');

d = orderfields(d);
fields = fieldnames(d);
for i=1:size(fields,1)
  d.(fields{i}) = (d.(fields{i}) -  init_utime) *1E-6
end

figure
for i=1:size(fields,1)
  plot(d.(fields{i}) , i,'+b')
  hold on 
end

set(gca,'YTick',[1:1:size(fields,1)])

set(gca,'YTickLabel', fields)

keyboard

function [d] = field_change(d,oldName,newName)
if isfield(d,oldName)
  [d.(newName)] = d.(oldName);
  d = rmfield(d,oldName);
else
  disp(['no field called ' oldName])
end