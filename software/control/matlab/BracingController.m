classdef BracingController < DRCController
    
    properties (SetAccess=protected,GetAccess=protected)
        robot;
    end
    
    methods
        function obj = BracingController(name,r,options)
            typecheck(r,'Atlas');
            
            ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1)));
            
            refpub = PositionRefPublisher(r,struct('gains_id','bracing'));
            
            % cascade qtraj eval block
            qt = QTrajEvalBlock(r,ctrl_data);
            ins(1).system = 1;
            ins(1).input = 1;
            outs(1).system = 2;
            outs(1).output = 1;
            sys = mimoCascade(qt,refpub,[],ins,outs);
            
            obj = obj@DRCController(name,sys,AtlasState(r));
            
            obj.robot = r;
            obj.controller_data = ctrl_data;
            
            obj = setTimedTransition(obj,20,name,false); % should transition to prone controller

            obj = addLCMTransition(obj,'WALKING_PLAN',drc.walking_plan_t(),'crawling');  % for crawling
            
        end
        
        function msg = status_message(obj,t_sim,t_ctrl)
          msg = drc.controller_status_t();
          msg.utime = t_sim * 1000000;
          msg.state = msg.BRACING;
          msg.controller_utime = t_ctrl * 1000000;
          msg.V = 0;
          msg.Vdot = 0;
        end        
        
        function obj = initialize(obj,data)
          
          % use saved nominal pose
          d = load('data/atlas_bracing.mat');
          q_nom = d.xstar(1:getNumDOF(obj.robot));

          if isfield(data,'AtlasState')
            x0 = data.AtlasState;
            q0 = x0(1:getNumDOF(obj.robot));

            qtraj = PPTrajectory(spline([0 1],[q0 q_nom]));
          else
            qtraj = q_nom;
          end
          
          obj.controller_data.setField('qtraj',qtraj);
  
        end
    end
    
end
