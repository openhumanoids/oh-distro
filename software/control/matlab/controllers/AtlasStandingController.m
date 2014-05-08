classdef AtlasStandingController < DRCController
  
  properties (SetAccess=protected,GetAccess=protected)
    robot;
    foot_idx;
  end
  
  methods
  
    function obj = AtlasStandingController(name,r,options)
      typecheck(r,'Atlas');

      if nargin < 3
        options = struct();
      end
      
      force_control_joint_str = {'leg'};% <---- cell array of (sub)strings  
      force_controlled_joints = [];
      for i=1:length(force_control_joint_str)
        force_controlled_joints = union(force_controlled_joints,find(~cellfun(@isempty,strfind(r.getInputFrame.coordinates,force_control_joint_str{i}))));
      end
      
      ctrl_data = SharedDataHandle(struct(...
        'is_time_varying',false,...
        'x0',zeros(4,1),...
        'support_times',0,...
        'supports',[],...
        'mu',1.0,...
        'trans_drift',[0;0;0],...
        'ignore_terrain',false,...
        'qtraj',zeros(getNumDOF(r),1),...
        'force_controlled_joints', force_controlled_joints,...
        'constrained_dofs',[findJointIndices(r,'arm');findJointIndices(r,'back');findJointIndices(r,'neck')]));
 
      sys = AtlasBalancingWrapper(r,ctrl_data,options);
      obj = obj@DRCController(name,sys,AtlasState(r));
 
      obj.robot = r;
      obj.controller_data = ctrl_data;
      obj.foot_idx = [r.findLinkInd('r_foot'),r.findLinkInd('l_foot')];

      % use saved nominal pose 
      d = load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
      q0 = d.xstar(1:getNumDOF(obj.robot));
      kinsol = doKinematics(obj.robot,q0);
      com = getCOM(obj.robot,kinsol);

      % build TI-ZMP controller 
      foot_pos = contactPositions(obj.robot,kinsol); 
      comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
      limp = LinearInvertedPendulum(com(3));
      K = lqr(limp,comgoal);
      
      supports = SupportState(r,[r.findLinkInd('r_foot'),r.findLinkInd('l_foot')]);
      
      obj.controller_data.setField('K',K);
      obj.controller_data.setField('qtraj',q0);
      obj.controller_data.setField('x0',[comgoal;0;0]);
      obj.controller_data.setField('y0',comgoal);
      obj.controller_data.setField('supports',supports);
      obj.controller_data.setField('firstplan',true);

      obj = addLCMTransition(obj,'START_MIT_STAND',drc.utime_t(),'stand');  
      obj = addLCMTransition(obj,'ATLAS_BEHAVIOR_COMMAND',drc.atlas_behavior_command_t(),'init'); 
      obj = addLCMTransition(obj,'CONFIGURATION_TRAJ',drc.configuration_traj_t(),name); % for standing/reaching tasks

    end
    
    function msg = status_message(obj,t_sim,t_ctrl)
        msg = drc.controller_status_t();
        msg.utime = t_sim * 1000000;
        msg.state = msg.STANDING;
        msg.controller_utime = t_ctrl * 1000000;
        msg.V = 0;
        msg.Vdot = 0;
    end
    
    function obj = initialize(obj,data)
       if isfield(data,'CONFIGURATION_TRAJ')
        % standing and reaching plan
        try
          msg = data.CONFIGURATION_TRAJ;
          obj.controller_data.setField('qtraj',mxDeserialize(msg.qtraj));
        catch err
          standAtCurrentState(obj,data.AtlasState);
        end
      elseif isfield(data,'AtlasState')
        standAtCurrentState(obj,data.AtlasState);
      end
      obj = setDuration(obj,inf,false); % set the controller timeout
    end
    
    function standAtCurrentState(obj,x0)
      r = obj.robot;
      q0 = x0(1:getNumDOF(r));
      kinsol = doKinematics(r,q0);
      foot_pos = contactPositions(r,kinsol,obj.foot_idx); 
      comgoal = mean([mean(foot_pos(1:2,1:4)');mean(foot_pos(1:2,5:8)')])';
      obj.controller_data.setField('qtraj',q0);
      obj.controller_data.setField('x0',[comgoal;0;0]);
      obj.controller_data.setField('y0',comgoal);
    end
  end
end
