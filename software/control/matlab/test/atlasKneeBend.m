function atlasKneeBend

% simple knee wiggle example using a sinusoid

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = AtlasPositionRef(r);

amp = pi/2.0;
period = 3.0;

qdes = zeros(getNumInputs(r),1);
r_knee_idx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_leg_kny')));

% execute knee bend
toffset=-1;
while 1
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
    end
    knee_q_des = -amp * (cos((t-toffset) * 2.0 * pi / period) - 1.0) / 2.0
    qdes(r_knee_idx) = knee_q_des;
    input_frame.publish(t,qdes,'ATLAS_COMMAND');
  end
end


end
