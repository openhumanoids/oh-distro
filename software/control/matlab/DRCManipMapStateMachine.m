classdef DRCManipMapStateMachine< handle

  properties (SetAccess=private,GetAccess=public)
    robot;
    qbreaks;
    r_hand_breaks;
    l_hand_breaks;
    r_foot_breaks;
    l_foot_breaks;
    qmap;
    affinds;
    mapindices;
    manip_map_received;
    l_hand_chain;
    r_hand_chain;
    l_foot_chain;
    r_foot_chain;
    chain_dofIndices;
    qcurrent;
  end

  methods
  
    function obj = DRCManipMapStateMachine(robot)
        typecheck(robot,'Atlas');
        obj.robot = robot;
        obj.manip_map_received=false;
        obj.qcurrent=zeros(getNumDOF(obj.robot),1);
    end
   
   function setMap(obj,xmap,affinds)
        obj.affinds = affinds;
        float_offset=6;
        obj.qbreaks =xmap(1:getNumDOF(obj.robot),:);
        obj.mapindices =  linspace(0,1,length(obj.affinds));
        l_hand_body = findLink(obj.robot,'l_hand+l_hand_point_mass');
        r_hand_body = findLink(obj.robot,'r_hand+r_hand_point_mass');
        l_foot_body = findLink(obj.robot,'l_foot');
        r_foot_body = findLink(obj.robot,'r_foot');
        pelvis_body = findLink(obj.robot,'pelvis');
        obj.l_hand_breaks=zeros(7,length(obj.mapindices));
        obj.r_hand_breaks=zeros(7,length(obj.mapindices));
        obj.l_foot_breaks=zeros(7,length(obj.mapindices));
        obj.r_foot_breaks=zeros(7,length(obj.mapindices));
        
        obj.l_hand_chain.dof_name = {};
        obj.l_hand_chain.dof_values = [];
        obj.r_hand_chain.dof_name = {};
        obj.r_hand_chain.dof_values = [];
        
        obj.l_foot_chain.dof_name = {};
        obj.l_foot_chain.dof_values = [];
        obj.r_foot_chain.dof_name = {};
        obj.r_foot_chain.dof_values = [];
        
        for brk =1:length(obj.mapindices),
            kinsol_tmp = doKinematics(obj.robot,obj.qbreaks(:,brk));
            pelvis_pose_worldframe= forwardKin(obj.robot,kinsol_tmp,pelvis_body,[0;0;0],2);
            l_hand_pose_worldframe= forwardKin(obj.robot,kinsol_tmp,l_hand_body,[0;0;0],2);
            r_hand_pose_worldframe= forwardKin(obj.robot,kinsol_tmp,r_hand_body,[0;0;0],2);
            l_foot_pose_worldframe= forwardKin(obj.robot,kinsol_tmp,l_foot_body,[0;0;0],2);
            r_foot_pose_worldframe= forwardKin(obj.robot,kinsol_tmp,r_foot_body,[0;0;0],2);
            
            
           rpy=quat2rpy(pelvis_pose_worldframe(4:7));
           % only take yaw into account. Ignore pelvis roll and pitch (to reflect affordance frame(car))
           T_world_body = obj.HT(pelvis_pose_worldframe(1:3),0*rpy(1),0*rpy(2),rpy(3));
           T_body_world = obj.inv_HT(T_world_body);
           % store ee break points in body frame (they should be stored in affordance frame)
            obj.l_hand_breaks(:,brk)= obj.applyHT(T_body_world,l_hand_pose_worldframe);
            obj.r_hand_breaks(:,brk)= obj.applyHT(T_body_world,r_hand_pose_worldframe);
            obj.l_foot_breaks(:,brk)= obj.applyHT(T_body_world,l_foot_pose_worldframe);
            obj.r_foot_breaks(:,brk)= obj.applyHT(T_body_world,r_foot_pose_worldframe);
            obj.l_hand_breaks(4:7,brk)=[1;0;0;0];
            obj.r_hand_breaks(4:7,brk)=[1;0;0;0];
            obj.l_foot_breaks(4:7,brk)=[1;0;0;0];
            obj.r_foot_breaks(4:7,brk)=[1;0;0;0];
            % poses are relative to pelvis
            %obj.l_hand_breaks(1:3,brk)=obj.l_hand_breaks(1:3,brk)-pelvis_pose(1:3);
            %obj.r_hand_breaks(1:3,brk)=obj.r_hand_breaks(1:3,brk)-pelvis_pose(1:3);
            %obj.l_foot_breaks(1:3,brk)=obj.l_foot_breaks(1:3,brk)-pelvis_pose(1:3);
            %obj.r_foot_breaks(1:3,brk)=obj.r_foot_breaks(1:3,brk)-pelvis_pose(1:3);
        end
        
        obj.qmap = PPTrajectory(spline(obj.mapindices,obj.qbreaks));
        
    
        % split aff indices into seperate structs for l/r hands and feet.
        % Basically treat as 4 separate maps for each ee and then merged the chains to form a final plan.
        % Q: will this work if the robot is not pinned?
        obj.l_hand_chain.active=false;obj.r_hand_chain.active=false;
        obj.l_foot_chain.active=false;obj.r_foot_chain.active=false;          
        for k =1:length(obj.affinds), % no of robot states in map
          for t =1:obj.affinds(k).num_ees, % no of ee's active per map pose, (fixed for entire map for now, eventually with a graph this wont be the case.)
            if (strcmp(char(obj.affinds(k).ee_name(t)),'l_hand')),
               obj.l_hand_chain.active=true;
               obj.l_hand_chain.dof_name(k) =obj.affinds(k).dof_name(t); % dof association
               obj.l_hand_chain.dof_values(k)=obj.affinds(k).dof_value(t);
            elseif (strcmp(char(obj.affinds(k).ee_name(t)),'r_hand')),
               obj.r_hand_chain.active=true;  
               obj.r_hand_chain.dof_name(k) =obj.affinds(k).dof_name(t); % dof association
               obj.r_hand_chain.dof_values(k)=obj.affinds(k).dof_value(t);
            elseif (strcmp(char(obj.affinds(k).ee_name(t)),'l_foot')),
                obj.l_foot_chain.active=true;  
                obj.l_foot_chain.dof_name(k) =obj.affinds(k).dof_name(t); % dof association
                obj.l_foot_chain.dof_values(k)=obj.affinds(k).dof_value(t);
            elseif (strcmp(char(obj.affinds(k).ee_name(t)),'r_foot')),
                obj.r_foot_chain.active=true;  
                obj.r_foot_chain.dof_name(k) =obj.affinds(k).dof_name(t); % dof association
                obj.r_foot_chain.dof_values(k)=obj.affinds(k).dof_value(t);
            end
          end
        end 
        
        
        
        obj.manip_map_received = true;
        obj.chain_dofIndices=[0 0 0];
        obj.determineDofIndices();
   end
   
    function setDofIndices(obj,dofIndices)
     % mapindices is a 3 element vector for each of the body chains
     % eventually this would set the active node in the graph
     % Set after Aff goal is received and plan is executing?
     % larm_index = argminimize ((qmap.getBreaks(larm)-qcurrent(larm))'(qmap.getBreaks(larm)-qcurrent(larm))) 
     obj.chain_dofIndices=dofIndices;
    end
    
    function determineDofIndices(obj)
     % mapindices is a 3 element vector for each of the body chains
     % eventually this would set the active node in the graph
     % Set after Aff goal is received and plan is executing?
     % larm_index = argminimize ((qmap.getBreaks(larm)-qcurrent(larm))'(qmap.getBreaks(larm)-qcurrent(larm))) 
      if(obj.manip_map_received),  
          kinsol = doKinematics(obj.robot,obj.qcurrent); 
          l_hand_body = findLink(obj.robot,'l_hand+l_hand_point_mass');
          r_hand_body = findLink(obj.robot,'r_hand+r_hand_point_mass');
          l_foot_body = findLink(obj.robot,'l_foot');
          r_foot_body = findLink(obj.robot,'r_foot');
          pelvis_body = findLink(obj.robot,'pelvis');
          
          current_l_hand_pose_worldframe= forwardKin(obj.robot,kinsol,l_hand_body,[0;0;0],2);
          current_r_hand_pose_worldframe= forwardKin(obj.robot,kinsol,r_hand_body,[0;0;0],2);
          current_l_foot_pose_worldframe= forwardKin(obj.robot,kinsol,l_foot_body,[0;0;0],2);
          current_r_foot_pose_worldframe= forwardKin(obj.robot,kinsol,r_foot_body,[0;0;0],2);
          current_pelvis_pose_worldframe= forwardKin(obj.robot,kinsol,pelvis_body,[0;0;0],2);
          
           % poses are relative to pelvis in body frame
           
          rpy=quat2rpy(current_pelvis_pose_worldframe(4:7));
          T_world_body = obj.HT(current_pelvis_pose_worldframe(1:3),0*rpy(1),0*rpy(2),rpy(3));
          T_body_world = obj.inv_HT(T_world_body);
          current_l_hand_pose_bodyframe = obj.applyHT(T_body_world,current_l_hand_pose_worldframe);
          current_r_hand_pose_bodyframe = obj.applyHT(T_body_world,current_r_hand_pose_worldframe);
          current_l_foot_pose_bodyframe = obj.applyHT(T_body_world,current_l_foot_pose_worldframe);
          current_r_foot_pose_bodyframe = obj.applyHT(T_body_world,current_r_foot_pose_worldframe);
          
          %current_l_hand_pose_bodyframe(1:3)=current_l_hand_pose_worldframe(1:3)-current_pelvis_pose_worldframe(1:3);
          %current_r_hand_pose_bodyframe(1:3)=current_r_hand_pose_worldframe(1:3)-current_pelvis_pose_worldframe(1:3);
          %current_l_foot_pose_bodyframe(1:3)=current_l_foot_pose_worldframe(1:3)-current_pelvis_pose_worldframe(1:3);
          %current_r_foot_pose_bodyframe(1:3)=current_r_foot_pose_worldframe(1:3)-current_pelvis_pose_worldframe(1:3);
          
        % this loop is slow.        
         tic; 
        finer_mapindices = 0:0.05:1;
        for i=1:length(finer_mapindices),
          si= finer_mapindices(i);
          % relative to pelvis
          r_hand_pose_at_ind=pose_foh(obj.mapindices,obj.r_hand_breaks,si); 
          l_hand_pose_at_ind=pose_foh(obj.mapindices,obj.l_hand_breaks,si); 
          r_foot_pose_at_ind=pose_foh(obj.mapindices,obj.r_foot_breaks,si); 
          l_foot_pose_at_ind=pose_foh(obj.mapindices,obj.l_foot_breaks,si); 
         
          
          % TODO: take into account orientation error.
          r_hand_err(i)=sum((r_hand_pose_at_ind(1:3)-current_r_hand_pose_bodyframe(1:3)).^2);
          l_hand_err(i)=sum((l_hand_pose_at_ind(1:3)-current_l_hand_pose_bodyframe(1:3)).^2);
          r_foot_err(i)=sum((r_foot_pose_at_ind(1:3)-current_r_foot_pose_bodyframe(1:3)).^2);
          l_foot_err(i)=sum((l_foot_pose_at_ind(1:3)-current_l_foot_pose_bodyframe(1:3)).^2);
        end
        toc;


        % get mapindices from current robot state
        [~,torso_index] = min(l_hand_err);
        if((obj.r_hand_chain.active)&&(~obj.l_hand_chain.active)) %  if right only, overide with rhand index
         [~,torso_index] = min(r_hand_err); 
        end
        [~,rleg_index] = min(r_foot_err); 
        [~,lleg_index] = min(l_foot_err); 
        obj.chain_dofIndices=finer_mapindices([torso_index,lleg_index,rleg_index]);
        %disp(obj.chain_dofIndices)
      end
    end
    
    function setActiveState(obj,qcurrent)
      obj.qcurrent = qcurrent;
    end
    
    function graphSearch(obj,relative_aff_goal)
    % TODO: given a relative aff goal, need to determine goal mapindices and the path taken in the graph to get to the goal.
    end

    function qtraj = getPlanGivenAffGoal(obj,aff_goal)  
            % is this going to be quick enough?
      qtraj_larm = []; % decoupled chains
      qtraj_rarm = [];
      qtraj_lleg = [];
      qtraj_rleg = [];
      qtraj=[];
      plan_length =25;
      valid_plans = 0;
      
      if(obj.manip_map_received), 
        obj.determineDofIndices();
        ee_names=cellstr(char(obj.affinds(1).ee_name));
        dof_names=cellstr(char(obj.affinds(1).dof_name)); % active dofs are fixed for a map (may not be in the future)
        
        for t=1:length(aff_goal.dof_name),
           % get active chain from aff_goal.dof_name(t)
           ind=find(strcmp(char(aff_goal.dof_name(t)),dof_names));
           end;
           if(strcmp(char(ee_names(ind)),'l_hand'))
               %disp('l_hand');
               start_mapindex=obj.chain_dofIndices(1);
               %start_mapindex=interp1(obj.l_hand_chain.dof_values,mapindices,aff_state.dof_value(t),'spline');
               desired_dof_value= min(max(aff_goal.dof_value(t),min(obj.l_hand_chain.dof_values)),max(obj.l_hand_chain.dof_values));
               goal_mapindex=interp1(obj.l_hand_chain.dof_values,obj.mapindices,desired_dof_value,'spline');
               start_mapindex = min(max(start_mapindex,0),1.0);
               goal_mapindex = min(max(goal_mapindex,0),1.0);
               plan_indices = linspace(start_mapindex,goal_mapindex,plan_length);
               %disp(obj.l_hand_chain.dof_values)
               %disp(aff_goal.dof_value(t))
               %disp(desired_dof_value)
               %disp(plan_indices)
               qtraj_larm = obj.qmap.eval(plan_indices);
               valid_plans = 1;
           elseif(strcmp(char(ee_names(ind)),'r_hand'))
               start_mapindex=obj.chain_dofIndices(1);
               %start_mapindex=interp1(obj.r_hand_chain.dof_values,mapindices,aff_state.dof_value(t),'spline');
               desired_dof_value= min(max(aff_goal.dof_value(t),min(obj.r_hand_chain.dof_values)),max(obj.r_hand_chain.dof_values));
               goal_mapindex=interp1(obj.r_hand_chain.dof_values,obj.mapindices,desired_dof_value,'spline'); 
               start_mapindex = min(max(start_mapindex,0),1.0);
               goal_mapindex = min(max(goal_mapindex,0),1.0);
               plan_indices = linspace(start_mapindex,goal_mapindex,plan_length);  
               qtraj_rarm = obj.qmap.eval(plan_indices);
               valid_plans = 1;
           elseif(strcmp(char(ee_names(ind)),'l_foot'))
               start_mapindex=obj.chain_dofIndices(2);
               %start_mapindex=interp1(obj.l_foot_chain.dof_values,mapindices,aff_state.dof_value(t),'spline');
               desired_dof_value= min(max(aff_goal.dof_value(t),min(obj.l_foot_chain.dof_values)),max(obj.l_foot_chain.dof_values));
               goal_mapindex=interp1(obj.l_foot_chain.dof_values,obj.mapindices,desired_dof_value,'spline');
               start_mapindex = min(max(start_mapindex,0),1.0);
               goal_mapindex = min(max(goal_mapindex,0),1.0);
               plan_indices = linspace(start_mapindex,goal_mapindex,plan_length);   
               qtraj_lleg = obj.qmap.eval(plan_indices);
               valid_plans = 1;
           elseif(strcmp(char(ee_names(ind)),'r_foot'))
               %disp('r_foot');
               start_mapindex=obj.chain_dofIndices(3);
               %start_mapindex=interp1(obj.r_foot_chain.dof_values,mapindices,aff_state.dof_value(t),'spline');
               desired_dof_value= min(max(aff_goal.dof_value(t),min(obj.r_foot_chain.dof_values)),max(obj.r_foot_chain.dof_values));
               goal_mapindex=interp1(obj.r_foot_chain.dof_values,obj.mapindices,desired_dof_value,'spline');
               start_mapindex = min(max(start_mapindex,0),1.0);
               goal_mapindex = min(max(goal_mapindex,0),1.0);
               plan_indices = linspace(start_mapindex,goal_mapindex,plan_length);                  
               %disp(plan_indices)
               qtraj_rleg = obj.qmap.eval(plan_indices);
               valid_plans = 1;
           end;
        end
        
        if(~valid_plans)
            qtraj = [];
            return;
        end;
        
        floatingoffset=6;
        
         %% This is 
        rarm =floatingoffset+[19  18  27  17  16  20];
        larm =floatingoffset+[7  6  15  5  4  8];
        back =floatingoffset+[1 2 3];
        neck =floatingoffset+[28];
        torso = [back neck larm rarm];
        lleg =floatingoffset+[12  14  11  10  13   9];
        rleg =floatingoffset+[24  26  23  22  25  21];
        
         %q_nominal = obj.qmap.eval(0);
         q_nominal = obj.qcurrent;
         %q0 = zeros(getNumDOF(obj.robot),1);  %
         qtraj = repmat(q_nominal,1,plan_length); % initialize to current state
         qtrajbak = qtraj;
         if(~isempty(qtraj_larm))
          qtraj(larm,:) = qtraj_larm(larm,:);
         end
         if(~isempty(qtraj_rarm))
          qtraj(rarm,:) = qtraj_rarm(rarm,:);
         end      
         if(~isempty(qtraj_lleg))
          qtraj(lleg,:) = qtraj_lleg(lleg,:);
         end     
         if(~isempty(qtraj_rleg))
          qtraj(rleg,:) = qtraj_rleg(rleg,:);
         end        
        
      end  % if(obj.manip_map_received)
    end  %end function 
  
  end % end methods
  
  methods (Static=true)
     function T= HT(p,roll,pitch,yaw)
        T = zeros(4);
        M = rotz(yaw)*roty(pitch)*rotx(roll);

        T(1:3,1:3) = M;
        T(1:4,4) = [p(:); 1];
      end
      function T_out=inv_HT(T)
          M = T(1:3,1:3);
          p = T(1:3,4);
          T_out = zeros(4);
          T_out(1:3,1:3) = M';
          p= -M'*p;
          T_out(1:4,4) = [p(:); 1];
      end
      function out_pose = applyHT(Frame_out_in,in_pose)
        rpy=quat2rpy(in_pose(4:7));
        Frame_in = DRCManipMapStateMachine.HT(in_pose(1:3),rpy(1),rpy(2),rpy(3));
        Frame_out = Frame_out_in*Frame_in;
        out_pose=0.*in_pose;
        out_pose(1:3) = Frame_out(1:3,4);
        out_pose(4:7) = rpy2quat(rotmat2rpy(Frame_out(1:3,1:3)));
      end

  end
  
end
