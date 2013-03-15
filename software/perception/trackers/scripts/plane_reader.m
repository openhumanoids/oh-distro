clear all;close all
% read the planes extracted from the plane reader   
fid=fopen('../data/car_simulator/car_planes.txt');
counter=1;
while 1
  tline = fgetl(fid);
  if ~ischar(tline), break, end
  
  strs = regexp(tline, ',', 'split');
  nums=[]
  for i=1:size(strs,2)
    nums(i) = str2num(strs{i});
  end
  n_pts = size(nums,2)/3;
  pts = reshape(nums, 3,n_pts)';
  planes{counter}.pts = pts;
  counter=counter+1;
end
fclose(fid);

colors = 'bgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmyk'

figure
hold on
for i=1:size(planes,2)
  
  plot3(planes{i}.pts(:,1), planes{i}.pts(:,2), planes{i}.pts(:,3), colors(i) )

end