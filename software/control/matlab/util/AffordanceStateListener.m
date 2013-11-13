classdef AffordanceStateListener<handle
  properties
    monitor;
    channel;
    aff_coders;
    aff_uid
    naffs;
    otdf_type;
  end
  methods
    function obj = AffordanceStateListener(channel)
      obj.channel = channel;
      obj.monitor = drake.util.MessageMonitor(drc.affordance_collection_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe(channel,obj.monitor);
      obj.aff_coders = {};
      obj.aff_uid = [];
      obj.naffs = 0;
      obj.otdf_type = {};
    end
    
    function [x,t] = getNextMessage(obj,t_ms)
      data = obj.monitor.getNextMessage(t_ms);
      if(isempty(data))
        x = [];
        t = -1;
      else
        msg = drc.affordance_collection_t(data);
        t = obj.monitor.getLastTimestamp();
        x = obj.decode(msg);
      end
    end

    function aff_data = decode(obj,msg)
      % @param uid     -- a 1 x naffs array, uid(i) is the uid for
      %                   affordance(i) 
      % @param state_frame -- a 1 x naffs cell, state_frame{i} is the state frame for
      %                       affordance(i)
      % @param otdf_type   -- a 1 x naffs cell, otdf_type{i} is the otdf_type for
      %                       affordance(i)
      msg_naffs = msg.naffs;
      msg_uid = zeros(1,msg_naffs);
      msg_otdf_type = cell(1,msg_naffs);
      msg_state_string = cell(1,msg_naffs);
      msg_state_frame = cell(1,msg_naffs);
      for i = 1:msg_naffs
        msg_uid(i) = msg.affs(i).uid;
        msg_otdf_type{i} = msg.affs(i).otdf_type;
        nq = msg.affs(i).nstates;
        msg_state_string{i} = msg.affs(i).state_names;
%         msg_state_string{i} = regexprep(cell(msg_state_string{i}),'::','_');
        msg_state_frame_coords = cell(2*nq,1);
        for j = 1:nq
          msg_state_frame_coords{j} = char(msg.affs(i).state_names(j));
          msg_state_frame_coords{j+nq} = strcat(char(msg.affs(i).state_names(j)),'dot');
        end
%         msg_state_frame_coords = regexprep(msg_state_frame_coords,'::','_');
        msg_otdf_type_rep = regexprep(char(msg_otdf_type{i}),'::','_');
        msg_state_frame{i} = CoordinateFrame(msg_otdf_type_rep,2*nq,[],msg_state_frame_coords);
      end
      
      [~,new_uid_idx] = setdiff(msg_uid,obj.aff_uid,'stable');
      [~,preserve_uid_idx] = intersect(obj.aff_uid,msg_uid,'stable');
      obj.aff_uid = [obj.aff_uid(preserve_uid_idx) msg_uid(new_uid_idx)];
      obj.naffs = length(obj.aff_uid);
      obj.aff_coders = obj.aff_coders(preserve_uid_idx);
      for i = 1:length(new_uid_idx)
        obj.aff_coders = [obj.aff_coders,{drc.control.AffordanceFullStateCoder(msg_otdf_type{new_uid_idx(i)},...
          java.lang.Integer(msg_uid(new_uid_idx(i))),msg_state_string{new_uid_idx(i)})}];
      end
      
      aff_data.state = cell(1,obj.naffs);
      aff_data.uid = obj.aff_uid;
      aff_data.otdf_type = msg_otdf_type;
      aff_data.rpy = zeros(3,obj.naffs);
      aff_data.xyz = zeros(3,obj.naffs);
      for i = 1:length(obj.aff_uid)
        fdata = obj.aff_coders{i}.decode(msg);
        aff_data.rpy(:,i) = [fdata.val(4);fdata.val(5);fdata.val(6)];
        aff_data.xyz(:,i) = [fdata.val(1);fdata.val(2);fdata.val(3)];
        nq = msg.affs(i).nstates;
        state_val = zeros(2*nq,1);
        for j = 1:nq
          state_val(j) = fdata.val(j+6);
          state_val(j+nq) = fdata.val(j+nq+12);
        end
        aff_data.state{i} = Point(msg_state_frame{i},state_val);
      end
    end
  end
end