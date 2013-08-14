function [xstar,ustar,zstar] = runCOMFixedPointSearch()

runsim = false; % run 1s sim with constant input after search

options.floating = true;
options.dt = 0.002;
urdf = strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf');

r = Atlas(urdf,options);
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);

v = r.constructVisualizer;
v.display_dt = options.dt*25;
dim=3;
x0 = Point(r.getStateFrame);

x0.base_z = 0.86000;
x0.base_pitch = 0.000000;
x0.back_bky = 0.000000;
x0.l_arm_elx = 1.000000;
x0.l_arm_ely = 1.870000;
x0.l_arm_shx = -1.300000;
x0.l_arm_usy = -0.0000;
x0.l_leg_hpy = -1.65000;
x0.l_leg_kny = 1.000000;cd
x0.l_leg_hpy = -0.650000;
x0.l_leg_aky = -0.35000;
x0.l_leg_hpx = 0.1000;
x0.l_leg_akx = -0.1000;

x0.r_arm_elx = -x0.l_arm_elx;
x0.r_arm_ely = x0.l_arm_ely;
x0.r_arm_shx = -x0.l_arm_shx;
x0.r_arm_usy = x0.l_arm_usy;
x0.r_leg_hpy = x0.l_leg_hpy;
x0.r_leg_kny = x0.l_leg_kny;
x0.r_leg_hpy = x0.l_leg_hpy;
x0.r_leg_aky = x0.l_leg_aky;
x0.r_leg_hpx = -x0.l_leg_hpx;
x0.r_leg_akx = -x0.l_leg_akx;

%x0 = resolveConstraints(r,double(x0));
v.draw(0,double(x0));
[xstar,ustar,zstar] = computeFixedPoint(r,double(x0),zeros(r.getNumInputs(),1),v);

if (runsim)
  % simulate fixed point 
  T = 1.0; % secdt
    
  c = ConstOrPassthroughSystem(ustar);
  c = setOutputFrame(c,r.getInputFrame);
  sys = cascade(c,r); 
  if (0)
      traj = simulate(sys,[0 T],xstar);
      playback(v,traj);
  else    
      s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
      sys = cascade(sys,v);
      warning(s);
      simulate(sys,[0 T],xstar);
  end
end

function [xstar,ustar,zstar] = computeFixedPoint(r,x0,u0,v)
  nx = r.getNumStates();
  nq = nx/2;
  nu = r.getNumInputs();
  nz = r.getNumContacts()*dim;
  z0 = zeros(nz,1);
  q0 = x0(1:nq);
    
  % take initial state guess as desired nominal pose
  qstar = x0(1:nq);
    
  problem.x0 = [q0;u0;z0];
  problem.objective = @(quz) myobj(quz);
  problem.nonlcon = @(quz) mycon(quz);
  problem.solver = 'fmincon';
  %problem.options=optimset('DerivativeCheck','on','GradObj','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-12);
  problem.options=optimset('GradObj','on','GradConstr','on','Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-6,'MaxFunEvals',1000);
  %problem.options=optimset('Algorithm','interior-point','Display','iter','OutputFcn',@drawme,'TolX',1e-6,'MaxFunEvals',2500);
      
  joint_names = r.getJointNames();
  joint_names = joint_names(2:end); % get rid of null string at beginning..
  r_idx = find(~cellfun(@isempty,strfind(joint_names,'r_leg')));
  l_idx = find(~cellfun(@isempty,strfind(joint_names,'l_leg')));
  x_idx = find(~cellfun(@isempty,strfind(joint_names,'x')));
  y_idx = find(~cellfun(@isempty,strfind(joint_names,'y')));
  z_idx = find(~cellfun(@isempty,strfind(joint_names,'z')));
  
  ry = intersect(r_idx,y_idx);
  ly = intersect(l_idx,y_idx);
  
  lb_z = -1e6*ones(nz,1);
  lb_z(dim:dim:end) = 0; % normal forces must be >=0
  ub_z = 1e6*ones(nz,1);
  [jl_min,jl_max] = r.getJointLimits();
  % force search to be close to starting position
  problem.lb = [max(q0-0.1,jl_min+0.01); r.umin; lb_z];
  problem.ub = [min(q0+0.1,jl_max-0.01); r.umax; ub_z];
  problem.lb(x_idx) = q0(x_idx);
  problem.ub(x_idx) = q0(x_idx);
  problem.lb(z_idx) = q0(z_idx);
  problem.ub(z_idx) = q0(z_idx);
  problem.lb(dim) = 0.8; % above the ground
  problem.ub(dim) = 1.2; % above the ground
    
  % fix foot starting position
  foot_pos = r.contactPositions(x0(1:nq));
  foot_pos(end,:) = 0; % we want the feet to be on the ground
  min_xf = min(foot_pos(1,:));
  max_xf = max(foot_pos(1,:));
  k = convhull(foot_pos(1:2,:)');
  com_des = [mean(foot_pos(1:2,k(1:end-1)),2);0];

  foot_pos_vec = reshape(foot_pos,nz,1);
    
  [quz_sol,~,exitflag] = fmincon(problem);
  xstar = [quz_sol(1:nq); zeros(nq,1)];
  ustar = quz_sol(nq+(1:nu));
  zstar = quz_sol(nq+nu+(1:nz));

  function stop=drawme(quz,optimValues,state)
    stop=false;
    v.draw(0,[quz(1:nq); zeros(nq,1)]);

    figure(66);
    plot(foot_pos(1,k),foot_pos(2,k),'bx','MarkerSize',10);
    hold on;
    plot(com_des(1),com_des(2),'go','MarkerSize',10);
    cm = r.getCOM(quz(1:nq));
    plot(cm(1),cm(2),'ro','MarkerSize',10);
    hold off;
  end
    
  function [f,df] = myobj(quz)
    q=quz(1:nq);
    [cm,J] = r.getCOM(q);
    W = diag([ones(1,dim-1),0]); % ignore z-component
    f = 0.5*(com_des-cm)'*W*(com_des-cm);
    df = [-(com_des-cm)'*W*J,zeros(1,nu+nz)];
  end

	function [c,ceq,GC,GCeq] = mycon(quz)
    q=quz(1:nq);
    u=quz(nq+(1:nu));
    z=quz(nq+nu+(1:nz));

%     [~,C,B,~,dC,~] = r.manipulatorDynamics(q,zeros(nq,1));
%     [cpos,J,dJ] = r.contactPositions(q);
    [cpos,J] = r.contactPositions(q);
    cpos = reshape(cpos,nz,1);

    % ignore friction constraints for now
    c = 0;
    GC = zeros(nq+nu+nz,1); 
% 
%     dJz = zeros(nq,nq);
%     for i=1:nq
%       dJz(:,i) = dJ(:,(i-1)*nq+1:i*nq)'*z;
%     end
    
    Jeq = zeros(length(ry),length(quz));
    Jeq(sub2ind(size(Jeq),1:length(ry),ry')) = ones(length(ry),1);
    Jeq(sub2ind(size(Jeq),1:length(ly),ly')) = -ones(length(ly),1);
%     ceq = [C-B*u-J'*z; cpos-foot_pos_vec; q(ry)-q(ly)];
%     GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[J'; zeros(nu+nz,length(cpos))],Jeq']; 
    ceq = [cpos-foot_pos_vec; q(ry)-q(ly)];
    GCeq = [[J'; zeros(nu+nz,length(cpos))],Jeq']; 
%     ceq = [C-B*u-J'*z; cpos-foot_pos];
%     GCeq = [[dC(1:nq,1:nq)-dJz,-B,-J']',[J'; zeros(nu+nz,length(cpos))]]; 
  end
end

end

