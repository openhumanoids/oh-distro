classdef StandingController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
  end
  
  methods
  
    function obj = StandingController(name,r)
      typecheck(r,'Atlas');

      ctrl_data = SharedDataHandle(struct('S',[],'h',[],'hddot',[],'qtraj',[],'supptraj',[],'ti_flag',true));
      
      % instantiate QP controller
      options.exclude_torso = false;
      options.slack_limit = 30.0;
      options.w = 0.25;
      options.R = 1e-12*eye(getNumInputs(r));
      qp = QPController(r,ctrl_data,options);

      % cascade PD qtraj controller 
%       Kp=diag([0;0;200;200;200;0;200*ones(getNumDOF(r)-6,1)]);
%       Kd=0.1*Kp;
      pd = SimplePDController(r,ctrl_data);
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
      
      obj = addLCMTransition(obj,'COMMITTED_WALKING_PLAN',drc.walking_plan_t(),'walking');

      % use saved nominal pose 
      d = load('data/atlas_fp.mat');
      xstar = d.xstar;
      nq = getNumDOF(r);
      
      q0 = xstar(1:nq);
      kinsol = doKinematics(r,q0);
      com = getCOM(r,kinsol);

      % build TI-ZMP controller 
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
  
      obj = setDuration(obj,inf,false); % set the controller timeout
      
    end
    
    function obj = initialize(obj,msg_data)

      % for now, use saved nominal pose, just update floating base
      % take in new nominal pose and compute standing controller
      if isfield(msg_data,'AtlasState')
        r = obj.robot;

        %d = load('data/atlas_fp.mat');
        %x0 = d.xstar;
        %x0([1;2;6]) = msg_data.AtlasState([1;2;6]);
        x0 = msg_data.AtlasState;
        q0 = x0(1:getNumDOF(r));
        kinsol = doKinematics(r,q0);
        com = getCOM(r,kinsol);

        % build TI-ZMP controller 
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
      
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
  end  
end
