classdef BracingController < DRCController
    
    properties (SetAccess=protected,GetAccess=protected)
        robot;
        head_monitor;
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
            
            
            obj.head_monitor = drake.util.MessageMonitor(sm.pose_t,'utime');
            obj.lc.subscribe('POSE_HEAD',obj.head_monitor);
            
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
          
          % Scott's code
          % use saved nominal pose 
          % d = load('data/atlas_bracing.mat');
          % q_nom = d.xstar(1:getNumDOF(obj.robot));
          
          %           if isfield(data,'AtlasState')
%             x0 = data.AtlasState;
%             q0 = x0(1:getNumDOF(obj.robot));
% 
%             qtraj = PPTrajectory(spline([0 1],[q0 q_nom]));
%           else
%             qtraj = q_nom;
%           end
% 
%          obj.controller_data.setField('qtraj',qtraj);
         

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          % Ani's code
          % Get torso position and velocities
          x0 = data.AtlasState;
          numq = getNumDOF(obj.robot);
          q0 = x0(1:numq);
          % pos = x0(1:3);
          % torso_dot = x0(numq+1:numq+6);
          % posdot = torso_dot(1:3);
          
          
          % Get head position, velocity and orientation
          %obj.head_monitor = drake.util.MessageMonitor(drc.pose_t,'utime');
          %obj.lc.subscribe('POSE_HEAD',obj.head_monitor);
          data = getNextMessage(obj.head_monitor);
          
          msg = sm.pose_t(data);
          % head_pos = msg.pos; % Position of head
          head_vel = msg.vel; % Velocity of head in local coordinates
          head_orient = msg.orientation; % Orientation of head (quaternion)
          
          % Transform velocity to correct coordinate frame
          Rt = quat2rotmat(head_orient);
          head_vel = Rt'*head_vel;
          
          % Get unit vector in direction of head's velocity
          uvec_head = head_vel/norm(head_vel);
          scale = 0.5; % Tune this by hand (distance from head)
          
          % Get head positions by forward kinematics
          kinsol = doKinematics(obj.robot,q0);
          head_body = obj.robot.findLinkInd('head');
          pos_head = forwardKin(obj.robot,kinsol,head_body,[0;0;0],0);
          
          xtarget = pos_head + scale*uvec_head; % Head position + scale*unit vector
          

          % Load Scott's file (I'm fixing the lower body joint angles to Scott's joint
          % angles)
          datfile = load('data/atlas_bracing.mat');
          qstar = datfile.xstar(1:getNumDOF(obj.robot)); % Scott's qnom
          
          nq = obj.robot.getNumDOF();
          coordinates = obj.robot.getStateFrame.coordinates;
          leg_ind = ~cellfun(@isempty,strfind(coordinates(1:nq),'leg'));
          options = struct();
          [options.jointLimitMin,options.jointLimitMax] = obj.robot.getJointLimits;
          options.jointLimitMin(leg_ind) = qstar(leg_ind);
          options.jointLimitMax(leg_ind) = qstar(leg_ind);
          
          % Do inverse kinematics to figure out target joint positions
          q_nom = inverseKin(obj.robot,q0,...
                    findLinkInd(obj.robot,'r_hand'),[0;0;0],xtarget, ...
                    findLinkInd(obj.robot,'l_hand'),[0;0;0],xtarget, ...
                    head_body,[0;0;0],xtarget,options);
                  
          % q_nom = inverseKin(obj,q0,body1,bodypos1,worldpos1,body2,bodypos2,worldpos2...,options)

          
          qtraj = PPTrajectory(spline([0 0.1],[q0 q_nom]));

          
          obj.controller_data.setField('qtraj',qtraj);
  
        end
    end
    
end
