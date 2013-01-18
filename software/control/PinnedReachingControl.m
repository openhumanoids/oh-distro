classdef PinnedReachingControl < DrakeSystem
% A simple jacobian-based reaching controller 
% NOTE: Hacky for now
  methods
    function obj = PinnedReachingControl(sys,r)
      % @param sys the PD-controlled closed-loop system
      % @param manip the manipulator, so I can call kinematics, etc. 
      
      typecheck(sys,'DrakeSystem');
      typecheck(r,'RigidBodyManipulator');
      
      obj = obj@DrakeSystem(0,sys.getNumInputs,sys.getNumStates,sys.getNumInputs,false,true);
      obj = setSampleTime(obj,[.01;0]); % update at 100 Hz
      obj = setInputFrame(obj,sys.getStateFrame);
      obj = setOutputFrame(obj,sys.getInputFrame);
      
      obj.manip = r;
      
      obj.rhand_ind = find(~cellfun(@isempty,strfind({obj.manip.body.linkname},'r_hand')));
      obj.lhand_ind = find(~cellfun(@isempty,strfind({obj.manip.body.linkname},'l_hand')));

      x0 = Point(r.getStateFrame);
      x0.l_arm_elx = 1.5;
      x0.l_arm_ely = 1.57;
      x0.l_arm_shx = -1.3;
      x0.r_arm_elx = -1.5;
      x0.r_arm_ely = 1.57;
      x0.r_arm_shx = 1.3;

      x0 = double(x0);
      obj.q_nom = x0(1:obj.manip.num_q);
      
      % arm indices
      obj.I_arms = zeros(obj.manip.num_q,1);
      obj.I_arms([5:9,16,17:21,28]) = ones(12,1);
      obj.I_arms = diag(obj.I_arms);
      
      B = obj.manip.getB();
      epsilon = 0.02;
      obj.q_d_max = obj.manip.joint_limit_max - epsilon;
      obj.q_d_min = obj.manip.joint_limit_min + epsilon;
      obj.q_d_max(obj.q_d_max == inf) = 1e10;
      obj.q_d_min(obj.q_d_min == -inf) = -1e10;
      obj.q_d_max = B' * obj.q_d_max;
      obj.q_d_min = B' * obj.q_d_min;
    end
    
    function q_d0 = getInitialState(obj)
      q_d0 = zeros(obj.manip.num_q,1);
    end
        
    function q_dn = update(obj,t,rep_des,lep_des,q_d,x)
      nq = obj.manip.num_q;
      q = x(1:nq);
      dt = 0.01;  % should really call getSampleTime

      kinsol = doKinematics(obj.manip,q); 
      [rep,Jrep] = forwardKin(obj.manip,kinsol,obj.rhand_ind,[0;-0.1;0]);
      [lep,Jlep] = forwardKin(obj.manip,kinsol,obj.lhand_ind,[0;0.1;0]);

      % desired right endpoint position
      k_ep = 1.5; % mfallon: multpiler on error
      err_rep = rep_des - rep;
      Jrep = Jrep*obj.I_arms;
      Nrep = eye(nq) - pinv(Jrep)*Jrep;
      dq_rep = k_ep *  pinv(Jrep) * err_rep;
      
      % desired left endpoint position
      err_lep = lep_des - lep;
      Jlep = Jlep*obj.I_arms;
      Nlep = eye(nq) - pinv(Jlep)*Jlep;
      dq_lep = k_ep * pinv(Jlep) * err_lep;
      
      [err_lep,err_rep]
     
      normbound = 1.0; % mfallon: norm bound to alleviate the effect of singularities.
      if (norm(dq_rep)>normbound)
        dq_rep = normbound*dq_rep/norm(dq_rep);
      end  
      if (norm(dq_lep)>normbound)
        dq_lep = normbound*dq_lep/norm(dq_lep);
      end  

      % compute nominal position error
      err_nom = obj.q_nom - q;
      k_nom = 0.5;
      dq_nom = k_nom * obj.I_arms * err_nom;
      
      dq_des = dq_rep + dq_lep + Nrep*Nlep*dq_nom;
      
      % map to input frame
      dq_des = obj.manip.getB()' * dq_des;
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
    I_arms
    q_d_max
    q_d_min
  end
end