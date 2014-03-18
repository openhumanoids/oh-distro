function atlasVisualizer
%NOTEST

% a visualizer process for visualizing atlas state and relevant variables

% load robot model
r = Atlas();
d=load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
xstar=d.xstar;
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
r = r.setInitialState(xstar);

% setup frames
state_plus_effort_frame = AtlasStateAndEffort(r);
state_plus_effort_frame.subscribe('EST_ROBOT_STATE');
force_torque_frame = AtlasForceTorque();
force_torque_frame.subscribe('EST_ROBOT_STATE');

l_foot_fz_idx = find(strcmp('l_foot_fz',force_torque_frame.coordinates));
l_foot_tx_idx = find(strcmp('l_foot_tx',force_torque_frame.coordinates));
l_foot_ty_idx = find(strcmp('l_foot_ty',force_torque_frame.coordinates));
r_foot_fz_idx = find(strcmp('r_foot_fz',force_torque_frame.coordinates));
r_foot_tx_idx = find(strcmp('r_foot_tx',force_torque_frame.coordinates));
r_foot_ty_idx = find(strcmp('r_foot_ty',force_torque_frame.coordinates));

nq = getNumDOF(r);
rfoot_ind = r.findLinkInd('r_foot');
lfoot_ind = r.findLinkInd('l_foot');

v = r.constructVisualizer;
lcmgl_com = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'center-of-mass');
lcmgl_cop = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'measured-cop');
lcmgl_zmp = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'filtered-zmp');

process_noise = 0.01*ones(nq,1);
observation_noise = 5e-4*ones(nq,1);
kf = FirstOrderKalmanFilter(process_noise,observation_noise);
kf_state = kf.getInitialState;


t_prev=-1;
qd_prev=-1;
qdd_prev=0;
alpha=0.05;
while true
  [x,t] = getNextMessage(state_plus_effort_frame,0);
  if ~isempty(x)
		if t_prev==-1
			dt=0.003;
			t_prev=t;
		elseif t_prev > t
			% skipped backwards in log, reset local state
			kf = FirstOrderKalmanFilter(process_noise,observation_noise);
			kf_state = kf.getInitialState;
			t_prev=-1;
			qd_prev=-1;
			qdd_prev=0;			
		else
			dt = t-t_prev;
			t_prev=t;
		end
		
% 		tau = x(2*nq+(1:nq));
 
		% get estimated state
    kf_state = kf.update(t,kf_state,x(1:nq));
    x_kf = kf.output(t,kf_state,x(1:nq));

    q_kf = x_kf(1:nq);
    qd_kf = x_kf(nq+(1:nq));
    
		if qd_prev==-1
			qdd = 0*qd_kf;
		else
			qdd = (1-alpha)*qdd_prev + alpha*(qd_kf-qd_prev)/dt;
		end
		qd_prev = qd_kf;
		qdd_prev = qdd;
		
    kinsol = doKinematics(r,q_kf,false,true);
		cpos = contactPositions(r,kinsol, [rfoot_ind, lfoot_ind]);
		ground_z = min(cpos(3,:));
		
		[com,J] = getCOM(r,kinsol);
		J = J(1:2,:); 

		drawZMP(kinsol,qd_kf,qdd,com,J,cpos,lcmgl_zmp);
		
		drawCOM(com,ground_z,lcmgl_com);
	
		force_torque = getMessage(force_torque_frame);
		drawCOP(force_torque,kinsol,lcmgl_cop);
		
    v.draw(t,x_kf);
  end
end

	function drawCOM(com,ground_z,lcmgl)
		lcmgl.glColor3f(0, 0, 0);
    lcmgl.sphere([com(1:2)', ground_z], 0.015, 20, 20);
    lcmgl.switchBuffers();
	end

	function drawZMP(kinsol,qd,qdd,com,J,cpos,lcmgl)
		Jdot = forwardJacDot(r,kinsol,0);
		Jdot = Jdot(1:2,:);
		
		% hardcoding D for ZMP output dynamics
		D = -1.04./9.81*eye(2); 

		comdd = Jdot * qd + J * qdd;
		zmp = com(1:2) + D * comdd;
		zmp = [zmp', min(cpos(3,:))];
		convh = convhull(cpos(1,:), cpos(2,:));
		zmp_ok = inpolygon(zmp(1), zmp(2), cpos(1,convh), cpos(2,convh));
		if zmp_ok
			color = [0 1 0];
		else
			color = [1 0 0];
		end
		lcmgl.glColor3f(color(1), color(2), color(3));
		lcmgl.sphere(zmp, 0.015, 20, 20);

    lcmgl.glColor3f(0, 0, 0);
    lcmgl.sphere([com(1:2)', min(cpos(3,:))], 0.015, 20, 20);

    lcmgl.switchBuffers();
	end

	function drawCOP(force_torque,kinsol,lcmgl)
		fz_l = force_torque(l_foot_fz_idx);
    tx_l = force_torque(l_foot_tx_idx);
		ty_l = force_torque(l_foot_ty_idx);
		l_foot_pt = [-ty_l/fz_l; tx_l/fz_l; 0];
    
		fz_r = force_torque(r_foot_fz_idx);
		tx_r = force_torque(r_foot_tx_idx);
		ty_r = force_torque(r_foot_ty_idx);
		r_foot_pt = [-ty_r/fz_r; tx_r/fz_r; 0];

		lfoot_pos = forwardKin(r,kinsol, lfoot_ind, l_foot_pt);
		rfoot_pos = forwardKin(r,kinsol, rfoot_ind, r_foot_pt);

		cop = (fz_l*lfoot_pos + fz_r*rfoot_pos)/(fz_l+fz_r);
		cop(3) = cop(3)-0.081;
		lcmgl.glColor3f(0,0,1);
		lcmgl.sphere(cop,0.015,20,20);
    lcmgl.switchBuffers();
	end


end