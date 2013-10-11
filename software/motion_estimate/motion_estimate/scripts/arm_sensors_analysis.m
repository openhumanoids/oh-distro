clear all; close all
arm_sensors=load('../src/state_sync/arm_sensors_300Hz');

joint_names = {'l-arm-usy', 'l-arm-shx', 'l-arm-ely', 'l-arm-elx', ...
               'l-arm-uwy', 'l-arm-mwx', 'r-arm-usy', 'r-arm-shx', ...
               'r-arm-ely', 'r-arm-elx', 'r-arm-uwy', 'r-arm-mwx'}


for joint=1:12
  i=joint+1;
  diffs(:,joint) = arm_sensors(:,i) - arm_sensors(:,i+12)
  
  mean_diffs(joint) = mean( diffs(:,joint) ) ;
  arm_sensors(:,i+12) = arm_sensors(:,i+12) + mean_diffs(joint);
  
  errors(:,joint) = arm_sensors(:,i) - arm_sensors(:,i+12);
end


figure; hold on
for joint=1:12
  i=joint+1
  subplot(3,4,joint)
  hold on
  plot(arm_sensors(:,i),'r.')
  plot(arm_sensors(:,i+12),'b.')
  axis tight
  title(joint_names{joint})
end

figure; hold on
for joint=1:12
  i=joint;
  subplot(3,4,joint)
  hold on
  plot(diffs(:,i),'r.')
  axis tight
  title(joint_names{joint})
end

figure; hold on
for joint=1:12
  i=joint;
  subplot(3,4,joint)
  hold on
  plot(errors(:,i),'k.')
  axis tight
  title(joint_names{joint})
end

mean_diffs