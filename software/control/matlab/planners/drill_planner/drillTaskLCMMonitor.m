classdef drillTaskLCMMonitor
  % NOTEST
  %
  % Class for monitoring incoming LCM messages related to the Drill task
  %  current responsibilities:
  %    -Get current state estimate
  %    -Get drill affordance information
  %    -Get wall affordance information
  properties
    lc
    affordance_monitor
    drill_control_monitor
    atlas
    state_frame
    hand_body;
  end
  
%   properties (Constant)
%     REFIT_DRILL = 1;            %Tell the planner to refit the drill affordance
%     RQ_NOMINAL_PLAN = 2;        %Find and transmit a trajectory for full drill task (and search for pose)
%     RQ_WALKING_GOAL = 3;        %Request the walking plan for the pose found in the step above
%     RQ_ARM_PREPOSE_PLAN = 4;    %Request a plan for moving the arm to a rough pre-drilling pose, before walking
%     RQ_NOMINAL_FIXED_PLAN = 5;  %Similar to nominal plan, but with fixed body pose
%     RQ_PREDRILL_PLAN = 6;       %Move the drill to a pre-drilling posture
%     RQ_DRILL_IN_PLAN = 7;       %Drill inward from the pre-drill posture
%     RQ_NEXT_DRILL_PLAN = 8;     %Get the next cut
%     RQ_DRILL_TARGET_PLAN = 9;   %Get a cut to [x,y,z] = data[1-3] in world frame
%     RQ_DRILL_DELTA_PLAN = 10;   %Get a cut to from current to offset by [x,y,z] = data[1-3] in wall frame (x-into, y-left, z-up)
%     
%     RQ_BUTTON_PREPOSE_PLAN = 11; %Get a plan for left and right arms to a good pre-button pose
%     RQ_BUTTON_DELTA_PLAN = 12;   %Get a plan for right arm+back to previous plan, and offset left finger by delta = data[1-3] in (TBD) frame
%   end
  
  methods
    function obj = drillTaskLCMMonitor(atlas, useRightHand)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.affordance_monitor = drake.util.MessageMonitor(drc.affordance_collection_t(), 'utime');
      obj.lc.subscribe('AFFORDANCE_COLLECTION',obj.affordance_monitor);
      
      obj.drill_control_monitor = drake.util.MessageMonitor(drc.drill_control_t(),'utime');
      obj.lc.subscribe('DRILL_CONTROL',obj.drill_control_monitor);
      
      obj.atlas = atlas;
      obj.state_frame = atlas.getStateFrame;
      obj.state_frame.subscribe('EST_ROBOT_STATE');
      
      
      if useRightHand
        obj.hand_body = regexpIndex('r_hand',{atlas.getBody(:).linkname});
      else
        obj.hand_body = regexpIndex('l_hand',{atlas.getBody(:).linkname});
      end
    end
    
    function [type, data] = getDrillControlMsg(obj)
      lcm_data = obj.drill_control_monitor.getNextMessage(2000);
      data = [];
      if isempty(lcm_data)
        type = -1;
        return;
      end
      
      msg = drc.drill_control_t(lcm_data);
      type = msg.control_type;
      data = msg.data;
    end
    
    function [wall_data, drill_data] = getWallAndDrillAffordances(obj)
      data = obj.affordance_monitor.getNextMessage(0); % default timeout
      if isempty(data)
        wall_data = [];
        drill_data = [];
        return
      end
      
      aff_collection = drc.affordance_collection_t(data);
      
      wall_aff = obj.getAffordanceByOTDFType(aff_collection, 'drill_wall');
      if isempty(wall_aff)
        wall_data = [];
      else
        wall_data = obj.parseWallData(wall_aff);
      end
      
      drill_aff = obj.getAffordanceByOTDFType(aff_collection, 'dewalt_button');
      if isempty(drill_aff)
        drill_data = [];
      else
        drill_data = obj.parseDrillData(drill_aff);
      end
    end
    
    function drill_data = getDrillAffordance(obj)
      data = obj.affordance_monitor.getNextMessage(0); % default timeout
      if isempty(data)
        drill_data = [];
        return
      end
      
      aff_collection = drc.affordance_collection_t(data);
      
      drill_aff = obj.getAffordanceByOTDFType(aff_collection, 'dewalt_button');
      if isempty(drill_aff)
        drill_data = [];
      else
        drill_data = obj.parseDrillData(drill_aff);
      end
    end
    
    function valve_data = getValveAffordance(obj)
      data = obj.affordance_monitor.getNextMessage(0); % default timeout
      if isempty(data)
        valve_data = [];
        return
      end
      
      aff_collection = drc.affordance_collection_t(data);
      
      valve_aff = obj.getAffordanceByOTDFType(aff_collection, 'steering_cyl');
      if isempty(valve_aff)
        valve_data = [];
      else
        valve_data = obj.parseValveData(valve_aff);
      end
    end
    
    function wall_data = getWallAffordance(obj)
      data = obj.affordance_monitor.getNextMessage(0); % default timeout
      if isempty(data)
        wall_data = [];
        return
      end
      
      aff_collection = drc.affordance_collection_t(data);
      
      wall_aff = obj.getAffordanceByOTDFType(aff_collection, 'drill_wall');
      if isempty(wall_aff)
        wall_data = [];
      else
        wall_data = obj.parseWallData(wall_aff);
      end
    end
    
    function q = getStateEstimate(obj)
      [lb, ub] = obj.atlas.getJointLimits;
      [x,~] = getNextMessage(obj.state_frame,10);
      while (isempty(x))
        [x,~] = getNextMessage(obj.state_frame,10);
      end
      q = x(1:34);
      
      if max(q - ub) > -1e-3,
        q = min(q,ub);
      end
      if max(lb - q) > -1e-3,
        q = max(q,lb);
      end
    end
  end
  
  
  methods (Access = private)
    
    % dewalt_button
    %  button_x,button_y,button_z
    %  button_nx, button_ny, button_nz
    %  guard_x, guard_y, guard_z
    %  guard_nx, guard_ny, guard_nz
    function drill_data = parseDrillData(obj, drill_aff)
      q = obj.getStateEstimate();
      kinsol = obj.atlas.doKinematics(q);
      % assuming right hand!
      hand_data = obj.atlas.forwardKin(kinsol,obj.hand_body,zeros(3,1),2);
      hand_pos = hand_data(1:3);
      R_hand = quat2rotmat(hand_data(4:7));
      
      R = rpy2rotmat(drill_aff.origin_rpy);
      drill_pos = drill_aff.origin_xyz;
      
      button_pos = zeros(3,1);
      button_normal = zeros(3,1);
      guard_pos = zeros(3,1);
      drill_axis = zeros(3,1);
      
      for i=1:drill_aff.nparams,
        pname = char(drill_aff.param_names(i));
        ind = unicode2native(pname(end)) - unicode2native('w');
        if regexp(pname,'button_[x-z]') == 1
          button_pos(ind) = drill_aff.params(i);
        elseif regexp(pname,'button_n[x-z]') == 1
          button_normal(ind) = drill_aff.params(i);
        elseif regexp(pname,'guard_[x-z]') == 1
          guard_pos(ind) = drill_aff.params(i);
        elseif regexp(pname,'guard_n[x-z]') == 1
          drill_axis(ind) = drill_aff.params(i);
        end
      end
      
      % transform into hand frame
      drill_data.button_pos = R_hand'*(R*button_pos + drill_pos - hand_pos);
      drill_data.button_normal = R_hand'*R*button_normal;
      drill_data.button_normal = drill_data.button_normal/norm(drill_data.button_normal);
      drill_data.guard_pos = R_hand'*(R*guard_pos + drill_pos - hand_pos);
      drill_data.drill_axis = R_hand'*R*drill_axis;
      drill_data.drill_axis = drill_data.drill_axis/norm(drill_data.drill_axis);
    end
    
    % positive-x points into the wall
    % look for drill
    % use otdf_type = 'drill_wall'
    % params are 'lx,ly,lz,p1y,p1z,p2y,p2z,p3y,p3z,...'
    function wall_data = parseWallData(obj, wall_aff)
      R = rpy2rotmat(wall_aff.origin_rpy);
      wall_data.normal = R(:,1);
      %       n_targets = (wall_aff.nparams - 3)/2;
      
      %       valuecheck(n_targets, floor(n_targets));  % check to see divisible by two
      n_values = 0; % for double checking
      
      %       points_on_wall = zeros(3,n_targets);
      
      for i=1:wall_aff.nparams,
        pname = char(wall_aff.param_names(i));
        if regexp(pname,'p[1-9][y-z]') == 1
          points_on_wall(unicode2native(pname(3)) - unicode2native('w'),str2num(pname(2))) = wall_aff.params(i);
          n_values = n_values + 1;
        end
      end
      n_targets = size(points_on_wall,2);
      if n_values ~= n_targets*2
        wall_data = [];
        send_msg = sprintf('Incorrect wall affordance. Expected %d target params, got %d', n_targets*2, n_values);
        send_status(4,0,0,send_msg);
        warning(send_msg);
        return;
      end
      
      % Transform wall points into world frame
      wall_data.targets = R*points_on_wall + repmat(wall_aff.origin_xyz,1,n_targets);
    end
    
    function valve_data = parseValveData(obj, valve_aff)
      R = rpy2rotmat(valve_aff.origin_rpy);
      valve_data.normal = -R(:,3);
      valve_data.radius = valve_aff.params(3) - .07; 
      valve_z = [0;0;1] - [0;0;1]'*valve_data.normal*valve_data.normal;
      valve_z = valve_z/norm(valve_z);
      valve_data.center = valve_aff.origin_xyz  + .10*valve_data.normal;
      valve_data.init_pt = valve_data.center + valve_z*valve_data.radius;
    end
    
    function aff = getAffordanceByOTDFType(obj, collection, type)
      aff = [];
      for i=1:collection.naffs,
        if strcmp(collection.affs(i).otdf_type,type)
          aff = collection.affs(i);
          return
        end
      end
    end
  end
  
end