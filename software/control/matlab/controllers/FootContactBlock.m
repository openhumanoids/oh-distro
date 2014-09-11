classdef FootContactBlock < MIMODrakeSystem

	properties
    contact_est_monitor;
    num_outputs;
    mex_ptr;
    controller_data;
    lfoot_idx;
    rfoot_idx;
    using_flat_terrain; % true if using DRCFlatTerrain
    contact_threshold; % min height above terrain to be considered in contact
    use_lcm;
    use_contact_logic_OR;
    robot;
    nq;
  end
  
  methods
    function obj = FootContactBlock(r,controller_data,options)
      typecheck(r,'Biped');
      typecheck(controller_data,'QPControllerData');
       
      if nargin<3
        options = struct();
      end
      
      % num_outputs option specifies how many copies of the output to
      % return
      if isfield(options,'num_outputs')
        typecheck(options.num_outputs,'double');
        sizecheck(options.num_outputs,[1 1]);
        rangecheck(options.num_outputs,1,inf);
      else
        options.num_outputs = 1;
      end
    
      input_frame = getStateFrame(r);
      if options.num_outputs > 1
        [outputs{1:options.num_outputs}] = deal(FootContactState);
        output_frame = MultiCoordinateFrame(outputs);
      else
        output_frame = FootContactState;
      end
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
			
      if isfield(options,'contact_threshold')
        % minimum height above terrain for points to be in contact
        typecheck(options.contact_threshold,'double');
        sizecheck(options.contact_threshold,[1 1]);
        obj.contact_threshold = options.contact_threshold;
      else
        obj.contact_threshold = 0.001;
      end
      
      if isfield(options,'use_lcm')
        % whether or not to listen for foot contact signals over LCM
        typecheck(options.use_lcm,'logical');
        sizecheck(options.use_lcm,[1 1]);
        obj.use_lcm = options.use_lcm;
      else
        obj.use_lcm = true;
      end

      if isfield(options,'use_contact_logic_OR')
        % false: always do a logical AND with planned support and sensed support
        % true: do logical OR with planned support and sensed support
        % except when breaking contact
        typecheck(options.use_contact_logic_OR,'logical');
        sizecheck(options.use_contact_logic_OR,[1 1]);
        obj.use_contact_logic_OR = options.use_contact_logic_OR;
      else
        obj.use_contact_logic_OR = false;
      end
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate

      obj.controller_data = controller_data;
      obj.num_outputs = options.num_outputs;
      
      if obj.use_lcm
        obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
        lc = lcm.lcm.LCM.getSingleton();
        if isfield(options,'channel')
          typecheck(options.channel,'char');
        else
          options.channel='FOOT_CONTACT_ESTIMATE';
        end
        lc.subscribe(options.channel,obj.contact_est_monitor);
      end
      
      terrain = getTerrain(r);
      if isa(terrain,'DRCTerrainMap') 
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end
      
      if exist('supportDetectmex','file')~=3
        error('can''t find supportDetectmex.  did you build it?');
      end      
      obj.mex_ptr = SharedDataHandle(supportDetectmex(0,r.getMexModelPtr.ptr,terrain_map_ptr));
  
      obj.rfoot_idx = findLinkInd(r,'r_foot');
      obj.lfoot_idx = findLinkInd(r,'l_foot');
          
      if isa(getTerrain(r),'DRCFlatTerrainMap')
        obj.using_flat_terrain = true;      
      else
        obj.using_flat_terrain = false;
      end
      obj.robot = r;
      obj.nq = getNumPositions(r);

    end
   
    function varargout=mimoOutput(obj,t,~,x)      
      ctrl_data = obj.controller_data;

      contact_logic_AND = true;
      if (ctrl_data.lqr_is_time_varying)
        % extract current desired supports
        supp_idx = find(ctrl_data.support_times<=t,1,'last');
        supp = ctrl_data.supports(supp_idx);      
        if obj.use_contact_logic_OR && supp_idx > 1
          supp_prev = ctrl_data.supports(supp_idx-1);
          breaking_contact = length(supp_prev.bodies)>length(supp.bodies);
          contact_logic_AND = breaking_contact;
        end  
      else      
        supp = ctrl_data.supports;
      end
      
      % contact_sensor = -1 (no info), 0 (info, no contact), 1 (info, yes contact)
      contact_sensor=-1+0*supp.bodies;  % initialize to -1 for all

      if obj.use_lcm
        % get foot contact state over LCM
        contact_data = obj.contact_est_monitor.getMessage(); % slow
        if ~isempty(contact_data)
          msg = drc.foot_contact_estimate_t(contact_data);
          contact_sensor(supp.bodies==obj.lfoot_idx) = msg.left_contact > 0.5;
          contact_sensor(supp.bodies==obj.rfoot_idx) = msg.right_contact > 0.5;
        end
      end      
      
      if ctrl_data.ignore_terrain
        contact_thresh =-1;       
      else
        contact_thresh = obj.contact_threshold;
      end
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end
      
      active_supports = supportDetectmex(obj.mex_ptr.data,x,supp,contact_sensor,contact_thresh,height,contact_logic_AND);
      y = [1.0*any(active_supports==obj.lfoot_idx); 1.0*any(active_supports==obj.rfoot_idx)];

      if ~y(1) 
        % left foot not in contact
        ind =  [ctrl_data.link_constraints.link_ndx]==obj.lfoot_idx;
        if ~isempty(ctrl_data.link_constraints(ind).contact_break_indices) && t>= ctrl_data.link_constraints(ind).ts(ctrl_data.link_constraints(ind).contact_break_indices(1))
          break_ind = ctrl_data.link_constraints(ind).contact_break_indices(1);
          q = x(1:obj.nq);
          qd = x(obj.nq+1:end);
          kinsol = doKinematics(obj.robot,q,false,true,qd);
          [p,J] = forwardKin(obj.robot,kinsol,obj.lfoot_idx,[0;0;0],1); 
          pdot = J*qd;
          
          ctrl_data.link_constraints(ind).ts(break_ind) = t;
          tf = ctrl_data.link_constraints(ind).ts(break_ind+1);
          pf = ctrl_data.link_constraints(ind).poses(:,break_ind+1);
          pfdot = ctrl_data.link_constraints(ind).dposes(:,break_ind+1);
          [a0, a1, a2, a3] = cubicSplineCoefficients(t, tf, p, pf, pdot, pfdot);
          ctrl_data.link_constraints(ind).poses(:,break_ind) = p;
          ctrl_data.link_constraints(ind).dposes(:,break_ind) = pdot;
          ctrl_data.link_constraints(ind).a0(:,break_ind) = a0;
          ctrl_data.link_constraints(ind).a1(:,break_ind) = a1;
          ctrl_data.link_constraints(ind).a2(:,break_ind) = a2;
          ctrl_data.link_constraints(ind).a3(:,break_ind) = a3;
          ctrl_data.link_constraints(ind).contact_break_indices(1) = [];

        end
%         if ~isempty(ctrl_data.link_constraints(ind).contact_break_times) && t >= ctrl_data.link_constraints(ind).contact_break_times(1)
%           q = x(1:obj.nq);
%           kinsol = doKinematics(obj.robot,q);
%           p = forwardKin(obj.robot,kinsol,obj.lfoot_idx,[0;0;0],1); 
%           ts = ctrl_data.link_constraints(ind).traj.getBreaks;
%           pts = ctrl_data.link_constraints(ind).traj.eval(ts);
%           ts(ctrl_data.link_constraints(ind).contact_break_ind(1)) = t;
%           pts(:,ctrl_data.link_constraints(ind).contact_break_ind(1)) = p;
%           new_traj = PPTrajectory(pchip(ts, pts));
%           ctrl_data.link_constraints(ind).traj = new_traj;
%           ctrl_data.link_constraints(ind).dtraj = fnder(new_traj);
%           ctrl_data.link_constraints(ind).contact_break_ind(1) = [];
%           ctrl_data.link_constraints(ind).contact_break_times(1) = [];
%         end
      elseif ~y(2)
        % right foot not in contact
        ind =  [ctrl_data.link_constraints.link_ndx]==obj.rfoot_idx;
        if ~isempty(ctrl_data.link_constraints(ind).contact_break_indices) && t>= ctrl_data.link_constraints(ind).ts(ctrl_data.link_constraints(ind).contact_break_indices(1))
          break_ind = ctrl_data.link_constraints(ind).contact_break_indices(1);
          q = x(1:obj.nq);
          qd = x(obj.nq+1:end);
          kinsol = doKinematics(obj.robot,q,false,true,qd);
          [p,J] = forwardKin(obj.robot,kinsol,obj.rfoot_idx,[0;0;0],1); 
          pdot = J*qd;
          
          ctrl_data.link_constraints(ind).ts(break_ind) = t;
          tf = ctrl_data.link_constraints(ind).ts(break_ind+1);
          pf = ctrl_data.link_constraints(ind).poses(:,break_ind+1);
          pfdot = ctrl_data.link_constraints(ind).dposes(:,break_ind+1);
          [a0, a1, a2, a3] = cubicSplineCoefficients(t, tf, p, pf, pdot, pfdot);
          ctrl_data.link_constraints(ind).poses(:,break_ind) = p;
          ctrl_data.link_constraints(ind).dposes(:,break_ind) = pdot;
          ctrl_data.link_constraints(ind).a0(:,break_ind) = a0;
          ctrl_data.link_constraints(ind).a1(:,break_ind) = a1;
          ctrl_data.link_constraints(ind).a2(:,break_ind) = a2;
          ctrl_data.link_constraints(ind).a3(:,break_ind) = a3;
          ctrl_data.link_constraints(ind).contact_break_indices(1) = [];
        end
%         if ~isempty(ctrl_data.link_constraints(ind).contact_break_times) && t >= ctrl_data.link_constraints(ind).contact_break_times(1)
%           q = x(1:obj.nq);
%           kinsol = doKinematics(obj.robot,q);
%           p = forwardKin(obj.robot,kinsol,obj.rfoot_idx,[0;0;0],1); 
%           ts = ctrl_data.link_constraints(ind).traj.getBreaks;
%           pts = ctrl_data.link_constraints(ind).traj.eval(ts);
%           ts(ctrl_data.link_constraints(ind).contact_break_ind(1)) = t;
%           pts(:,ctrl_data.link_constraints(ind).contact_break_ind(1)) = p;
%           new_traj = PPTrajectory(pchip(ts, pts));
%           ctrl_data.link_constraints(ind).traj = new_traj;
%           ctrl_data.link_constraints(ind).dtraj = fnder(new_traj);
%           ctrl_data.link_constraints(ind).contact_break_ind(1) = [];
%           ctrl_data.link_constraints(ind).contact_break_times(1) = [];
%         end
      end
      
      if obj.num_outputs > 1
        varargout = cell(1,obj.num_outputs);
        for i=1:obj.num_outputs
          % looping seems faster than using deal or arrayfun
          varargout{i} = y;
        end
      else
        varargout = {y};
      end
		end
  end
  
end
