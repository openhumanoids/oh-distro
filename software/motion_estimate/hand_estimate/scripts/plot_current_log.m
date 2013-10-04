ros-string -> ros-py -> ros-joint-command   (all individual commands)
lcm-string -> lcm-py -> lcm-joint-command   (all individual commands)



give commanded joint angles



close  all; clear all
log = load('sandia_current_log')



win = 100

moving_weight = 0.97

for finger = 2:5
  moving = 0;
  for i=win+1:size(log,1)
    log_mean(i,finger ) = mean(log(i-win:i,finger ));
    
    moving = moving_weight*moving + (1- moving_weight) *log(i,finger);
    log_moving(i,finger ) = moving;
  end
  
  
end

figure
subplot(3,1,1)
plot( log(:,[2:5]) )

subplot(3,1,2)
plot( log_mean(:,[2:5]) )

subplot(3,1,3)
plot( log_moving(:,[2:5]) )