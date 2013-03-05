classdef SimpleZMPTracker < DrakeSystem
   
  methods
    function obj = SimpleZMPTracker(r,x0,dZMP)
      typecheck(r,'Atlas');
      typecheck(dZMP,'PPTrajectory');
      
      input_frame = getStateFrame(r);
      output_frame = AtlasCOM(r);

      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      addpath(fullfile(getDrakePath,'examples','ZMP'));
      
      nq = getNumDOF(r);
      q0 = x0(1:nq);
      kinsol = doKinematics(r,q0);
      com = getCOM(r,kinsol);
            
      limp = LinearInvertedPendulum(com(3));
      [obj.c, obj.V] = ZMPtracker(limp,dZMP);
      obj.manip = r;
      obj.com0 = com;
    end
   
    function y = output(obj,t,~,x)
      nq = getNumDOF(obj.manip);
      q = x(1:nq); qd = x(nq+1:end);
      
%       disp('simple zmp');
%       x
      kinsol = doKinematics(obj.manip,q);
      [com,J] = getCOM(obj.manip,kinsol);
      com_dot = J*qd;
      
      com_xy_ddot = obj.c.output(t,0,[com(1:2);com_dot(1:2)]);

%       % also do PD control on the COM height
%       Kp = 200; 
%       Kd = 40;
%       err = obj.com0(3) - com(3);
%       com_z_ddot = Kp*err - Kd*com_dot(3);
%       y = [com_xy_ddot; com_z_ddot];
      y = [com_xy_ddot; 0];
    end
  end
  
  properties
    manip
    c
    com0
    V % Lyapunov candidate
  end
end