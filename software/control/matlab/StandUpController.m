classdef StandUpController < DRCController
    
    properties (SetAccess=protected,GetAccess=protected)
        robot;
    end
    
    methods
        function obj = StandUpController(name,r,options)
            typecheck(r,'Atlas');
            
            ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1)));
            
            refpub = PositionRefPublisher(r,struct('gains_id','crawling'));
            
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
            
            obj = setTimedTransition(obj,48,name,false); 
            
            obj = addLCMTransition(obj,'STOP_CRAWLING',drc.utime_t(),'all_fours');
        end
        
        function msg = status_message(obj,t_sim,t_ctrl)
          msg = drc.controller_status_t();
          msg.utime = t_sim * 1000000;
          msg.state = msg.CRAWLING;
          msg.controller_utime = t_ctrl * 1000000;
          msg.V = 0;
          msg.Vdot = 0;
        end        
        
        function obj = initialize(obj,data)
          
          % use saved nominal pose
          d = load('data/atlas_stand_up.mat');
          q_nom = d.x_nom(1:getNumDOF(obj.robot));
          q_fall = d.x_fall(1:getNumDOF(obj.robot));
          t_seq = 4*d.t_fine_sequence;
          q_seq = d.x_fine_sequence(1:getNumDOF(obj.robot),:);

          if isfield(data,'AtlasState')
            x0 = data.AtlasState;
            q0 = x0(1:getNumDOF(obj.robot));

            qtraj = PPTrajectory(foh(t_seq,q_seq));
          else
            qtraj = q_nom;
          end
          
          obj.controller_data.setField('qtraj',qtraj);
  
        end
    end
    
end
