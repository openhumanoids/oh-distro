function make_plot()

close all
task = 'drill'

data_path = '/home/mfallon/data/atlas/2013-12-21-competition/analysis'

if ( strcmp(task,'debris'))
  init_utime = 1387544165992342
elseif (strcmp(task,'drill'))
  % log:
  %init_utime = 1387562271287314
  % mission:
  init_utime = 1387562807870000
elseif (strcmp(task,'walking_1'))
  init_utime = 1387640095497967
elseif (strcmp(task,'walking_2'))
  init_utime =   1387656689696912
elseif (strcmp(task,'hose'))
  init_utime =   1387551234326817
elseif (strcmp(task,'climbing'))
  init_utime = 1387644558521343
elseif (strcmp(task,'valves'))
  init_utime = 1387555533539921
elseif (strcmp(task,'doors'))
  init_utime = 1387568748959451  
elseif (strcmp(task,'driving'))
  init_utime = 1387631421120009
end




f=figure
set(gcf, 'Position', [3000, 100, 640, 600]);

 h= subplot(4,1,[1 2])
 if ( strcmp(task,'drill'))
   make_gantt(h,data_path,task)
 end
 
 h= subplot(4,1,3)
 if ( isempty(strfind(task,'walking')))
   make_activity(h,data_path,task,init_utime)
 end

h= subplot(4,1,4)
make_bw(f,h,task,data_path,init_utime) % to robot

%h= subplot(5,1,5)
%results = make_bw(f,h,task,init_utime,0)% from robot
%ylabel('KB/s from Robot')