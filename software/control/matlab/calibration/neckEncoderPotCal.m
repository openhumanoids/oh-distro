%NOTEST
if ~exist('r')
  r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'));
end
logfile = strcat(getenv('DRC_PATH'),'/../logs/lcmlog-2013-10-15.00_neck_encoder_pot');
[t_x,x_data,t_u,u_data,~,~,state_frame,input_frame,t_extra,extra_data] = parseAtlasViconLog(r,logfile,14);
t_extra = t_extra(1:end-1);
extra_data = extra_data(:,1:end-1);

t_u = t_u - t_x(1);
t_extra = t_extra - t_x(1);
t_x = t_x - t_x(1);