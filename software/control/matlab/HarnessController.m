classdef HarnessController < DRCController
    
    properties (SetAccess=protected,GetAccess=protected)
        robot;
        floating;
    end
    
    methods
        function obj = HarnessController(name,r,timeout)
            typecheck(r,'Atlas');
            
            ctrl_data = SharedDataHandle(struct('qtraj',zeros(getNumDOF(r),1)));
            
            % instantiate QP controller
            options = struct();
            options.R = 1e-12*eye(getNumInputs(r));
            qp = HarnessQPController(r,options);
            
            % cascade PD controller
            if getNumDOF(r)==34 % floating model
                options.Kp=diag([zeros(6,1); 200*ones(getNumDOF(r)-6,1)]);
                float = true;
            else
                options.Kp=diag(200*ones(getNumDOF(r),1));
                float = false;
            end
            
            options.Kd=0.12*options.Kp;
            pd = SimplePDController(r,ctrl_data,options);
            ins(1).system = 1;
            ins(1).input = 1;
            ins(2).system = 2;
            ins(2).input = 2;
            outs(1).system = 2;
            outs(1).output = 1;
            sys = mimoCascade(pd,qp,[],ins,outs);
            
            % cascade neck pitch control block
            if(~strcmp(name,'seated_driving'))
                neck = NeckControlBlock(r,ctrl_data);
                ins(1).system = 1;
                ins(1).input = 1;
                ins(2).system = 1;
                ins(2).input = 2;
                ins(3).system = 2;
                ins(3).input = 2;
                outs(1).system = 2;
                outs(1).output = 1;
                connection.from_output = 1;
                connection.to_input = 1;
                sys = mimoCascade(neck,sys,connection,ins,outs);
            end
            
            obj = obj@DRCController(name,sys);
            
            obj.robot = r;
            obj.controller_data = ctrl_data;
            obj.floating = float;
            
            if nargin < 3
                if((strcmp(name,'harnessed_notwalking'))||(strcmp(name,'seated_driving')))
                    % controller timeout must match the harness time set in mit_not_walking.launch
                    obj = setTimedTransition(obj,inf,'standing',true);
                else
                    % controller timeout must match the harness time set in mit.launch
                    obj = setTimedTransition(obj,10,'standing',true);
                end
            else
                obj = setTimedTransition(obj,timeout,'standing',true);
            end
            
            obj = addLCMTransition(obj,'COMMITTED_ROBOT_PLAN',drc.robot_plan_t(),name);
            obj = addPrecomputeResponseHandler(obj,'STANDING_PREC_RESPONSE','standing');
            
            % add precompute trigger
            obj = addPrecomputeTrigger(obj,@obj.standingRequestTrigger);
            
        end
        
        function trigger_active = standingRequestTrigger(obj,input_data,times)
            trigger_active = true;
            t=max(times);
            if ~isinf(getDuration(obj)) && t>4.0
                x = input_data{2};
                if(strcmp(obj.name,'seated_driving'))
                    d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_seated_pose.mat'));%hands down
                    %d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_seated.mat'));%hands up
                else
                    d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
                end
                q_nom = d.xstar(1:getNumDOF(obj.robot));
                q_nom([1,2,6]) = x([1,2,6]); % copy over pelvix x,y,yaw
                q_nom(3) = x(3)-0.02; % z is harness pelvis height, which is slightly higher than standing pelvis height
                
                disp('HarnessController:publishing standing precompute request.');
                req_msg = drc.precompute_request_t();
                req_msg.utime = 1000000*t;
                req_msg.robot_name = 'atlas';
                req_msg.response_channel = 'STANDING_PREC_RESPONSE';
                req_msg.precompute_type = 0;
                
                save('prec_r.mat','q_nom');
                fid = fopen('prec_r.mat','r');
                req_msg.matdata = fread(fid,inf,'*uint8');
                fclose(fid);
                req_msg.n_bytes = length(req_msg.matdata);
                lc = lcm.lcm.LCM.getSingleton();
                lc.publish('STANDING_PREC_REQUEST', req_msg);
                trigger_active = false; % run once
            end
        end
        
        
        function obj = initialize(obj,data)
            
            if isfield(data,'COMMITTED_ROBOT_PLAN')
                % pinned reaching plan
                msg = getfield(data,'COMMITTED_ROBOT_PLAN');
                [xtraj,ts] = RobotPlanListener.decodeRobotPlan(msg,true);
                if obj.floating
                    qtraj = PPTrajectory(spline(ts,xtraj(1:getNumDOF(obj.robot),:)));
                else
                    qtraj = PPTrajectory(spline(ts,xtraj(6+(1:getNumDOF(obj.robot)),:)));
                end
                obj = setDuration(obj,inf,false); % set the controller timeout
            else
                % use saved nominal pose
                if(strcmp(obj.name,'seated_driving'))
		  d =load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_seated_pose.mat'));%hands down
                  %d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_seated.mat'));%hands up
               else
                    d = load('data/atlas_fp.mat');
                end
                
                if ~obj.floating
                    q_nom = d.xstar(6+(1:getNumDOF(obj.robot)));
                else
                    q_nom = d.xstar(1:getNumDOF(obj.robot));
                end
                q0 = zeros(getNumDOF(obj.robot),1);
                if((strcmp(obj.name,'harnessed_notwalking'))||(strcmp(obj.name,'seated_driving')))
                    qtraj = PPTrajectory(spline([0 4],[q0 q_nom]));
                else
                    qtraj = PPTrajectory(spline([5 9],[q0 q_nom]));
                end
            end
            
            obj.controller_data.setField('qtraj',qtraj);
            
        end
    end
    
end
