dirname = '/home/drc/drcdata/robotiq/2014-03-21-drill2-modeling/jpg';

listing = dir(fullfile(dirname, ...
               '*.jpg'));
for i = 1:length(listing)
  filepath = fullfile(dirname, listing(i).name);
  disp(filepath);
  IM = imread(filepath);
  edgeandshow(IM);
  pause(0.01)
end