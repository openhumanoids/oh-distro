function publishRobotDebris()

xstar= [0,0,0,0,0,0,1,-0.246950149536, 0.00316667556763, -0.00521075725555, 1.14958667755, 0.00421768426895, 0.0511291027069, -0.804708003998, 1.56003177166, -0.770609498024, -0.0488710962236, -0.0078706741333, -0.0163278579712, -0.775656223297, 1.54737138748, -0.783533096313, 0.0399675630033, 0.255834490061, -1.30554103851, 2.036921978, 0.455057352781, 0.053745046258, -0.0125637603924, 0.30805721879, 1.37250006199, 2.04537343979, -0.403616964817, 0.00682938098907, -1.05751430988]

%%%%%


%%%%%%%
pos = xstar(1:3)
quat = xstar(4:7)
joint_position = xstar(8:35)
joint_name ={'back_bkz', 'back_bky', 'back_bkx', ...
         'neck_ay', 'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', ...
         'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'r_leg_hpz', ...
         'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', ...
         'r_leg_akx', 'l_arm_usy', 'l_arm_shx', 'l_arm_ely', ...
         'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx', 'r_arm_usy', ...
         'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx'}

       
pos =[ 1.76 -0.2 1]
quat = [0.8805 0 0 -0.47]
quat = quat ./norm(quat)


trans= drc.vector_3d_t();
trans.x= pos(1); trans.y= pos(2); trans.z= pos(3)
rot= drc.quaternion_t();
rot.w = quat(1); rot.x = quat(2); rot.y = quat(3); rot.z = quat(4);
pose= drc.position_3d_t();
pose.translation = trans;
pose.rotation= rot;

linv= drc.vector_3d_t();
angv= drc.vector_3d_t();
twist= drc.twist_t();
twist.linear_velocity = linv;
twist.angular_velocity = angv;

ft= drc.force_torque_t();

ers= drc.robot_state_t();
ers.pose = pose;
ers.twist = twist;
ers.force_torque = ft;

ers.joint_position = joint_position
ers.joint_velocity = zeros(28,1)
ers.joint_effort = zeros(28,1)
ers.joint_name = joint_name

ers.num_joints = 28

lc = lcm.lcm.LCM.getSingleton();
lc.publish('EST_ROBOT_STATE', ers);


%send_status()

%
%publishRobotStateContinuously(xstar,'EST_ROBOT_STATE')



function send_status()
ers= drc.robot_state_t();
ers.utime  = etime(clock,[1970 1 1 0 0 0])*1000000;
ers.num_joints =0
lc = lcm.lcm.LCM.getSingleton();
lc.publish('SYSTEM_STATUS', ers);
%pause(0.25)


function publishRobotStateContinuously(x, channel)
  r = RigidBodyManipulator('');
  
  r = r.addRobotFromURDF( strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'), [],[],struct('floating',true));
  joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
  robot_state_coder = LCMCoordinateFrameWCoder('AtlasState',r.getNumStates(), ...
    'x',JLCMCoder(drc.control.RobotStateCoder(joint_names)));

  starttime = cputime;

  while (1)
  	pause(0.01);
  	publish(robot_state_coder,0,x,channel);
  end
