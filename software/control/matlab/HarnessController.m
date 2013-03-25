classdef HarnessController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end  
    
  methods
    function obj = HarnessController(name,r)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('S',[],'h',[],'hddot',[],'qtraj',[],'supptraj',[],'ti_flag',true));
      
      % instantiate QP controller
      options = struct();
      options.R = 1e-12*eye(getNumInputs(r));
      qp = HarnessQPController(r,options);

      % cascade PD controller 
      Kp=diag([zeros(6,1);100*ones(getNumDOF(r)-6,1)]);
      Kd=0.2*Kp;
      pd = SimplePDController(r,ctrl_data,Kp,Kd);
      ins(1).system = 1;
      ins(1).input = 1;
      ins(2).system = 2;
      ins(2).input = 2;
      outs(1).system = 2;
      outs(1).output = 1;
      sys = mimoCascade(pd,qp,[],ins,outs);

      obj = obj@DRCController(name,sys);

      obj.robot = r;
      obj.controller_data = ctrl_data;

      obj = setTimedTransition(obj,10,'standing',true); % set the controller timeout to 10s simtime
    end
    
    function obj = initialize(obj,msg_data)

      r = obj.robot;
      nq = getNumDOF(r);
      
      % use saved nominal pose --- could make this more general
      d = load('data/atlas_fp.mat');
      xstar = d.xstar;
    
      jn = getJointNames(r);
      xstar(cellfun(@isempty,strfind(jn(2:end),'arm'))) = 0.0; 
      
      q0 = xstar(1:nq);
      kinsol = doKinematics(r,q0);
      com = getCOM(r,kinsol);

      % build TI-ZMP controller ... kinda silly to do this for the harness
      foot_pos = contactPositions(r,q0); 
      ch = convhull(foot_pos(1:2,:)'); % assumes foot-only contact model
      comgoal = [mean(foot_pos(1:2,ch),2);com(3)];
      limp = LinearInvertedPendulum(com(3));
      options.Qy = diag([0 0 0 0 1 1]);
      [~,V] = tilqr(limp,Point(limp.getStateFrame,[comgoal(1:2);0;0]), ...
                    Point(limp.getInputFrame),zeros(4),zeros(2),options);

      foot_support=1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot'));

      obj.controller_data.setField('S',V.S);
      obj.controller_data.setField('h',com(3));
      obj.controller_data.setField('hddot',0);
      obj.controller_data.setField('qtraj',q0);
      obj.controller_data.setField('supptraj',foot_support);
  
    end
  end  
end
