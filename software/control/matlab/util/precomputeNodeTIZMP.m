function controller_data = precomputeNodeTIZMP(r,req)
  % r is the robot model
  % req is lcm message of type precompute_request_t
  % TIZMP precompute node assumes that the matdata field contains: 
  %    q_nom OR 
  %    qtraj and t_f (in which case the TI controller will be
  %    calcuated for qtraj.eval(t_f)) OR
  %    q0, comtraj, lfoottraj, rfoottraj, t_f
          
  fid = fopen('prec_w.mat','w');
  fwrite(fid,typecast(req.matdata,'uint8'),'uint8');
  fclose(fid);
  matdata = load('prec_w.mat');
  if isfield(matdata,'q_nom')
    q_nom=matdata.q_nom;
  elseif isfield(matdata,'qtraj')
    if isfield(matdata,'t_f')
      t_f = matdata.t_f;
    else
      t_f = matdata.qtraj.tspan(2);
    end
    q_nom = matdata.qtraj.eval(t_f);
  else
    % no q trajectory, so compute qtraj from foottraj and comtraj
    if isfield(matdata,'ikoptions')
      ikoptions = matdata.ikoptions;
    else
      % setup IK parameters
      cost = Point(r.getStateFrame,1);
      cost.base_x = 0;
      cost.base_y = 0;
      cost.base_z = 0;
      cost.base_roll = 1000;
      cost.base_pitch = 1000;
      cost.base_yaw = 0;
      cost.back_lbz = 10;
      cost.back_mby = 100;
      cost.back_ubx = 100;

      cost = double(cost);
      nq = getNumDOF(r);
      ikoptions = struct();
      ikoptions.Q = diag(cost(1:nq));
      d=load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      ikoptions.q_nom = d.xstar(1:nq);
    end

    if isfield(matdata,'t_f')
      t_f = matdata.t_f;
    else
      t_f = matdata.comtraj.tspan(2);
    end
    
    rfoot_body = r.findLink(r.r_foot_name);
    lfoot_body = r.findLink(r.l_foot_name);

    q=q0;
    for t=0:0.1:t_f
      q = approximateIK(r,q,0,[matdata.comtraj.eval(t);nan], ...
        rfoot_body,[0;0;0],matdata.rfoottraj.eval(t), ...
        lfoot_body,[0;0;0],matdata.lfoottraj.eval(t),ikoptions);
    end
    
    q_nom = q;
  end

  kinsol = doKinematics(r,q_nom);
  com = getCOM(r,kinsol);

  % build TI-ZMP controller 
  foot_pos = contactPositions(r,q_nom); 
  ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
  comgoal = mean(foot_pos(1:2,ch),2);
  limp = LinearInvertedPendulum(com(3));
  [~,V] = lqr(limp,comgoal);

  foot_support=1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot'));

  controller_data = struct();
  controller_data.S=V.S;
  controller_data.h=com(3);
  controller_data.hddot=0;
  controller_data.q_nom=q_nom;
  controller_data.support=foot_support;
  controller_data.xlimp0=[comgoal;0;0];
  controller_data.ti_flag=true;
       
end
