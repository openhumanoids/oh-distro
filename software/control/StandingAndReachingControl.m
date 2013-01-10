classdef StandingAndReachingControl < DrakeSystem
% A simple jacobian-based standing and reaching controller 
%
  methods
    function obj = StandingAndReachingControl(sys,r)
      % @param sys the PD-controlled closed-loop system
      % @param manip the manipulator, so I can call kinematics, etc. 
      
      typecheck(sys,'DrakeSystem');
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      obj = obj@DrakeSystem(0,sys.getNumInputs,sys.getNumStates,sys.getNumInputs,false,true);
      obj = setSampleTime(obj,[.01;0]); % update at 100 Hz
      obj = setInputFrame(obj,sys.getStateFrame);
      obj = setOutputFrame(obj,sys.getInputFrame);
      
      obj.manip = r.manip;
      
      obj.rhand_ind = find(~cellfun(@isempty,strfind({r.manip.body.linkname},'r_hand')));
      obj.lhand_ind = find(~cellfun(@isempty,strfind({r.manip.body.linkname},'l_hand')));

      x0 = Point(r.getStateFrame);
      x0.l_arm_elx = 1.5;
      x0.l_arm_ely = 1.57;
      x0.l_arm_shx = -1.3;
      x0.r_arm_elx = -1.5;
      x0.r_arm_ely = 1.57;
      x0.r_arm_shx = 1.3;

      x0 = double(x0);
      obj.q_nom = x0(7:obj.manip.num_q);
      
      % arm joint index matrices
      obj.I_right_arm = diag([zeros(1,22),ones(1,5),zeros(1,7)]);
      obj.I_left_arm = diag([zeros(1,10),ones(1,5),zeros(1,19)]);

      epsilon = 0.01;
      obj.q_d_max = obj.manip.joint_limit_max - epsilon;
      obj.q_d_min = obj.manip.joint_limit_min + epsilon;
      
      obj.q_d_max(obj.q_d_max == inf) = 1e10;
      obj.q_d_min(obj.q_d_min == -inf) = -1e10;

      obj.q_d_max = r.manip.B' * obj.q_d_max;
      obj.q_d_min = r.manip.B' * obj.q_d_min;
    end
    
    function q_d0 = getInitialState(obj)
      q_d0 = zeros(obj.manip.num_q,1);
    end
        
    function q_dn = update(obj,t,rep_des,lep_des,q_d,x)
      nq = obj.manip.num_q;
      q = x(1:nq);
      dt = 0.01;  % should really call getSampleTime

      [gc,Jgc] = obj.manip.contactPositions(q);
      [cm,Jcm] = obj.manip.getCOM(q);
      kinsol = doKinematics(obj.manip,q); 
      [rep,Jrep] = forwardKin(obj.manip,kinsol,obj.rhand_ind,[0;0;0]);
      [lep,Jlep] = forwardKin(obj.manip,kinsol,obj.lhand_ind,[0;0;0]);

      % compute desired COM projection
      P = diag([1 1 0]);
      k = convhull(gc(1:2,:)');
      cm_des = [mean(gc(1:2,k),2);0];

      Jp = Jgc(:,1:6);
      Ja = Jgc(:,7:end);
      Pq_qa = [-pinv(Jp)*Ja;eye(nq-6)];
      
      % get ground contact nullspace projection matrix
      Jc = Jgc * Pq_qa;
      Ngc = eye(nq-6) - pinv(Jc)*Jc;
      
      % compute COM error 
      err_com = cm_des - P*cm;
      J_com = P*Jcm*Pq_qa;
      k_com = 0.15;
      dq_com = k_com * pinv(J_com) * err_com;
 
      % COM nullspace projection matrix
      Ncom = eye(nq-6) - pinv(J_com)*J_com;

      % desired right endpoint position
      k_ep = 0.375;
      err_rep = rep_des - rep;
      Jrep = Jrep*obj.I_right_arm*Pq_qa;
      Nrep = eye(nq-6) - pinv(Jrep)*Jrep;
      dq_rep = k_ep * pinv(Jrep) * err_rep;

      % desired left endpoint position
      err_lep = lep_des - lep;
      Jlep = Jlep*obj.I_left_arm*Pq_qa;
      Nlep = eye(nq-6) - pinv(Jlep)*Jlep;
      dq_lep = k_ep * pinv(Jlep) * err_lep;

      normbound = 0.5; % norm bound to alleviate the effect of singularities.
      if (norm(dq_rep)>normbound)
        dq_rep = normbound*dq_rep/norm(dq_rep);
      end  
      if (norm(dq_lep)>normbound)
        dq_lep = normbound*dq_lep/norm(dq_lep);
      end  

      % compute nominal position error
      err_nom = obj.q_nom - q(7:end);
      k_nom = 0.2;
      dq_nom = k_nom * err_nom;
      
      % do null space projections and map into input coordinates
      if (t<2.0)% || t>15.75) 
        dq_des = Ngc * (dq_com + Ncom*dq_nom);
      else
        dq_des = Ngc * (dq_com + Ncom*(dq_rep + dq_lep) + Ncom*Nrep*Nlep*dq_nom);
      end
      
      % debug
      %valuecheck(norm(Jgc*Pq_qa*dq_des),0);

      % map to input frame
      dq_des = obj.manip.B(7:end,:)' * dq_des;

      q_dn = q_d + dt*dq_des;
      
      q_dn = min(max(q_dn,obj.q_d_min),obj.q_d_max);
    end
    
    function y = output(obj,t,q_d,x)
      y = q_d;
    end
  end
  
  properties
    manip
    rhand_ind
    lhand_ind
    q_nom
    I_right_arm
    I_left_arm
    q_d_max
    q_d_min
  end
end