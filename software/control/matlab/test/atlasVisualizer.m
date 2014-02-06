function atlasVisualizer
%NOTEST

% a visualizer process for visualizing atlas state and relevant variables

% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
r = r.setInitialState(xstar);

% setup frames
state_plus_effort_frame = AtlasStateAndEffort(r);
state_plus_effort_frame.subscribe('EST_ROBOT_STATE');

nq = getNumDOF(r);
rfoot_ind = r.findLinkInd('r_foot');
lfoot_ind = r.findLinkInd('l_foot');

v = r.constructVisualizer;
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'atlas-visualizer');

process_noise = 0.01*ones(nq,1);
observation_noise = 5e-4*ones(nq,1);
kf = FirstOrderKalmanFilter(process_noise,observation_noise);
kf_state = kf.getInitialState;

% hardcoding D for ZMP output dynamics
D = -0.89./9.81*eye(2); 

t_prev=-1;
qd_prev=-1;
qdd_prev=0;
alpha=0.05;
while true
  [x,t] = getNextMessage(state_plus_effort_frame,1);
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

		[com,J] = getCOM(r,kinsol);
    Jdot = forwardJacDot(r,kinsol,0);
		J = J(1:2,:); 
		Jdot = Jdot(1:2,:);
		
		comdd = Jdot * qd_kf + J * qdd;
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
    v.draw(t,x_kf);
  end
end

end