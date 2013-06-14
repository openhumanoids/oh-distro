classdef SitUpController < DRCController
    
    properties (SetAccess=protected,GetAccess=protected)
        robot;
        floating;
        state_frame;
    end
    
    methods
        function obj = SitUpController(name,r,options)
            typecheck(r,'Atlas');
            
            %ctrl_data =
            %SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1)));
            %d = ...
            %    load(strcat(getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_seated.mat'))
            %q_desired = ...
            %    d.xstar(1: ...
            %            getNumDOF(r));
            
            x0 = getInitialState(r); 
            q0 = x0(1:getNumDOF(r));
            %q0
            ctrl_data = SharedDataHandle(struct('qtraj',q0));
            %This sets everything
            %to zero 
            
            %obj.state_frame = r.getStateFrame();
            %obj.state_frame.subscribe('EST_ROBOT_STATE');
            
            refpub = PositionRefPublisher(r,struct('gains_id','sit_up'));
            
            % cascade qtraj eval block
            qt = QTrajEvalBlock(r,ctrl_data);
            ins(1).system = 1;
            ins(1).input = 1;
            outs(1).system = 2;
            outs(1).output = 1;
            sys = mimoCascade(qt,refpub,[],ins,outs);
            
            obj = obj@DRCController(name,sys,AtlasState(r));
            
            obj.floating = false;
            
            obj.robot = r;
            obj.controller_data = ctrl_data;
            
            obj = setTimedTransition(obj,inf,name,false); % should transition to prone controller
            obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name); % self-transition
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
            fprintf(1, 'Init Called - Situp controller\n');
%             x0 = getInitialState(obj.robot); 
%             q0 = x0(1:getNumDOF(robot));
%             q0
            if isfield(data,'COMMITTED_ROBOT_PLAN')
                % pinned reaching plan
                msg = data.COMMITTED_ROBOT_PLAN;
                joint_names = obj.robot.getStateFrame.coordinates(1:getNumDOF(obj.robot));
                [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,obj.floating,joint_names);
                % We don't need to incorporate the offset for floating as the
                % RobotPlanListener takes care of it
                qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
                obj = setDuration(obj,inf,false); % set the controller timeout
                
                fprintf (1, 'should be publishing qtraj');
                obj.controller_data.setField('qtraj',qtraj);
                
            else
                state_frame = obj.robot.getStateFrame();
                state_frame.subscribe('EST_ROBOT_STATE');
                while true
                    [x0,ts] = getNextMessage(state_frame,5);
                    if (~isempty(x0))
                        q0 = x0(1:getNumDOF(obj.robot));
                        obj.controller_data.setField('qtraj',q0);
                        break;
                    end;
                end;                   
            end;
            
            %obj = setDuration(obj,inf,false); 
        end
    end
    
end
