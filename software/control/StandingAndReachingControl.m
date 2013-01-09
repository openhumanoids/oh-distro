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
    end
    
    function q_d0 = getInitialState(obj)
      q_d0 = zeros(obj.manip.num_q,1);
    end
        
    function q_dn = update(obj,t,ep_des,q_d,x)
      nq = obj.manip.num_q;
      q = x(1:nq);
      dt = 0.01;  % should really call getSampleTime

      [gc,Jgc] = obj.manip.contactPositions(q);
      [cm,Jcm] = obj.manip.getCOM(q);
      kinsol = doKinematics(obj.manip,q); 
      [ep,Jep] = forwardKin(obj.manip,kinsol,obj.rhand_ind,[0;0;0]);

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
      k_com = 0.2;
      dq_com = k_com * pinv(J_com) * err_com;
 
      % COM nullspace projection matrix
      Ncom = eye(nq-6) - pinv(J_com)*J_com;

      % desired right endpoint position
      err_ep = ep_des - ep;
      k_ep = 0.35;
      Jep = Jep*obj.I_right_arm*Pq_qa;
      dq_ep = k_ep * pinv(Jep) * err_ep;
      %[ep_des,ep]
      normbound = 1.0; % norm bound to alleviate the effect of singularities.
      if (norm(dq_ep)>normbound)
        dq_ep = normbound*dq_ep/norm(dq_ep);
      end  


      % endpoint nullspace projection matrix
      Nep = eye(nq-6) - pinv(Jep)*Jep;

      % compute nominal position error
      err_nom = obj.q_nom - q(7:end);
      k_nom = 0.2;
      dq_nom = k_nom * err_nom;
      
      % do null space projections and map into input coordinates
      if (t<5.0)% || t>15.75) 
        dq_des = Ngc * (dq_com + Ncom*dq_nom);
      else
        dq_des = Ngc * (dq_com + Ncom*dq_ep + Ncom*Nep*dq_nom);
      end
      
      % debug
      %valuecheck(norm(Jgc*Pq_qa*dq_des),0);

      % map to input frame
      dq_des = obj.manip.B(7:end,:)' * dq_des;

      q_dn = q_d + dt*dq_des;
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
  end
end