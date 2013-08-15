function atlasManipulateTest
%NOTEST

options.floating = true;
options.dt = 0.002;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);

state_frame = getStateFrame(r);
state_frame.subscribe('EST_ROBOT_STATE');
input_frame = AtlasPositionRef(r);

nu = getNumInputs(r);
nq = getNumDOF(r);
period = 4.0;
ramp_time = 5.0;

qdes = zeros(nu,1);
r_elx_idx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'r_arm_elx')));
l_elx_idx = find(~cellfun(@isempty,strfind(input_frame.coordinates,'l_arm_elx')));

toffset=-1;
q0=zeros(nq,1);
act_idx = getActuatedJoints(r);
while 1
  [x,t] = getNextMessage(state_frame,1);
  if ~isempty(x)
    if toffset==-1
      toffset=t;
      q0 = x(1:nq);
      qdes=q0(act_idx);
    end
    tt=t-toffset;
    
    % Value use to interpolate from initial joint angles to desired
		% joint angles
    s = max(min(1.0, tt / ramp_time), 0.0);
    amp = s*pi/2.0;
    
    % Create a sinusoidal motion for the elbows
    % Desired elbow angle
		elx_q_d = -amp * (cos(tt * 2.0 * pi / period) - 1.0) / 2.0;

    % Note: if in atlas manip behavior mode, the desired values for leg
    % joints are ignored
    qdes(r_elx_idx) = (1-s) * q0(r_elx_idx) - s*elx_q_d;
    qdes(l_elx_idx) = (1-s) * q0(l_elx_idx) + s*elx_q_d;
    					
    r_elx_command=qdes(r_elx_idx)
    r_elx_command=qdes(l_elx_idx)

    input_frame.publish(t,qdes,'ATLAS_COMMAND');
  end
end


end
