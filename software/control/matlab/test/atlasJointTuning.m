function atlasJointTuning
%NOTEST

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = getInputFrame(r);
ref_frame = AtlasTorquePosRef(r);

nu = getNumInputs(r);

l_arm_usy = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_usy')));
l_arm_shx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_shx')));
l_arm_ely = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_ely')));
l_arm_elx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_elx')));
l_arm_uwy = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_uwy')));
l_arm_mwx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_mwx')));

r_arm_shx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_shx')));


gains = getAtlasGains(input_frame);

gains.k_f_p = zeros(nu,1);

% tune shx joint
gains.k_q_p(l_arm_shx) = 0;
gains.k_q_p(r_arm_shx) = 0;

gains.k_f_p(l_arm_shx) = 0.105;
gains.ff_f_d(l_arm_shx) = 0.0;
gains.ff_qd(l_arm_shx) = 0.0;

u=zeros(nu,1);
qdes=zeros(nu,1);

ref_frame.updateGains(gains);

toffset = -1;
udes = PPTrajectory(foh([0 10 20 30],[60 0 60 0]));
while 1
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
  
    u(l_arm_shx) = udes.eval(t-toffset);

    ref_frame.publish(t,[qdes;u],'ATLAS_COMMAND');
  end
end

end
