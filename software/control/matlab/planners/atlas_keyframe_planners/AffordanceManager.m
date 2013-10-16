classdef AffordanceManager < handle
  % @param num_affs            -- A scalar, the total number of affordances
  % @param aff_uid             -- A 1 x num_affs array. aff_uid(i) is the uid of
  %                               affordance(i)
  % @param aff_state           -- A 1 x num_affs cell, aff_state{i} is the state of
  %                               affordance(i)
  % @param aff_xyz             -- A 3 x num_affs array, aff_xyz(:,i) is the position of
  %                               the base of affordance(i)
  % @param aff_rpy             -- A 3 x num_affs array, aff_rpy(:,i) is the rpy angle of
  %                               the base of affordiance(i)
  % @param isCollision         -- A 1 x num_affs boolean array, isCollision(i) = true if
  %                               affordance(i) is a collision object
  % @param aff_state_listener  -- The listener to affordance channel
  % @param aff2robotFrameMap   -- A 1 x num_affs cell. aff2robotFrameMap{i}(j) records the
  %                               state index of affordance{i} state(j) in the Robot
  %                               frame. If affordance(i) is not a part of the robot,
  %                               aff2robotFrameMap{i}(j) = -1 for all j
  % @param atlas2robotFrameMap -- atlas2robotFrameMap(i) is the index of atlas state(i) in
  %                               robot state frame.
  properties
    aff_state_listener
    num_affs;
    aff_uid;
    aff_state;
    aff_xyz;
    aff_rpy;
    isCollision
    aff2robotFrameMap
    atlas2robotFrameMap
  end
  
  methods
    function obj = AffordanceManager(atlas,channel)
      obj.aff_state_listener = AffordanceStateListener(channel);
      obj.num_affs = 0;
      obj.aff_uid = [];
      obj.aff_state = {};
      obj.aff_xyz = [];
      obj.aff_rpy = [];
      obj.isCollision = [];
      obj.aff2robotFrameMap = {};
      obj.atlas2robotFrameMap = (1:atlas.getNumStates)';
    end
    
    function updateWmessage(obj,t_ms)
      msg = obj.aff_state_listener.getNextMessage(t_ms);
      if(~isempty(msg))
        cache_aff_uid = obj.aff_uid;
        obj.aff_uid = msg.uid;
        obj.num_affs = length(obj.aff_uid);
        obj.aff_state = msg.state;
        obj.aff_xyz = msg.xyz;
        obj.aff_rpy = msg.rpy;
        cache_isCollision = obj.isCollision;
        obj.isCollision = false(1,obj.num_affs);
        [~,cache_uid_idx, new_uid_idx] = intersect(cache_aff_uid,obj.aff_uid);
        obj.isCollision(new_uid_idx) = cache_isCollision(cache_uid_idx);
      end
    end
    
    function updateWcollisionObject(obj,robot_frame,aff_uid,atlas_frame)
      % @param robot_frame     -- CoordinateFrame of robot, including both atlas and
      %                           collision affordance
      % @param aff_uid         -- An array of the affordance uid that are newly added as
      %                           collision object
      % @param atlas_frame     -- CoordinateFrame of Atlas
      obj.atlas2robotFrameMap = zeros(length(atlas_frame.coordinates),1);
      for i = 1:length(atlas_frame.coordinates)
        obj.atlas2robotFrameMap(i) = find(cellfun(@(x) strcmp(atlas_frame.coordinates{i},x),robot_frame.coordinates));
      end
      for i = 1:length(obj.aff_uid)
        aff_frame_i = obj.aff_state{i}.frame;
        obj.aff2robotFrameMap{i} = zeros(length(aff_frame_i.coordinates),1);
        for j = 1:length(aff_frame_i.coordinates)
          aff2robotFrameMapIdx = find(cellfun(@(x) strcmp(aff_frame_i.coordinates{j},x),robot_frame.coordinates));
          if(isempty(aff2robotFrameMapIdx))
            obj.aff2robotFrameMap{i}(j) = -1;
          else
            obj.aff2robotFrameMap{i}(j) = aff2robotFrameMapIdx;
          end
        end
      end
      % update isCollisionFlag
      [~,aff_uid_idx] = intersect(obj.aff_uid,aff_uid);
      obj.isCollision(aff_uid_idx) = true;
    end
  end
end