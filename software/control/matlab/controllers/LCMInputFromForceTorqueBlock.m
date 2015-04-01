classdef LCMInputFromForceTorqueBlock < MIMODrakeSystem
  
  properties
    lc;
    lcmonitor_cmd; %LCM monitors
    r;
    r_control;
    
    last_wrench;
    
    bodyid;
  end
  
  methods
    function obj = LCMInputFromForceTorqueBlock(r, r_control, bodyname, options)
      typecheck(r,'Atlas');
      % r_control is an atlas model with a state of just
      % atlas_state.
      if ~isempty(r_control)
        typecheck(r_control, 'Atlas');
      else
        r_control = r;
      end
      
      if nargin<4
        options = struct();
      end

      % Generate AtlasInput as out (we'll do translation manually)
      output_frame = r.getInputFrame.getFrameByName('three_dof_forceInput');
      
      % We want atlasstate as input so we can apply force in global frame
      input_frame = drcFrames.AtlasState(r_control);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      obj.r_control = r_control;
      obj.bodyid = obj.r_control.getManipulator.findLinkId(bodyname);
      
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.lcmonitor_cmd = drake.util.MessageMonitor(drake.lcmt_force_torque,'timestamp');
      obj.lc.subscribe('SIM_WRENCH_INPUT',obj.lcmonitor_cmd);
      
      obj.last_wrench = SharedDataHandle([0;0;0]);
    end
    
    function varargout=mimoOutput(obj,t,~,atlas_state)
      
      % see if we have a new message (new command state)
      data = obj.lcmonitor_cmd.getMessage();
      if (~isempty(data))
        data = drake.lcmt_force_torque(data);
        nq = obj.r_control.getNumPositions;
        q = atlas_state(1:nq);
        qd = atlas_state(nq+1:end);
        kinsol = doKinematics(obj.r_control,q);
        x = forwardKin(obj.r_control, kinsol, obj.bodyid, [0;0;0], 1);
        rpy = x(4:6);
        rpy(3) = 0; % Force should yaw with robot
        quat_world_to_body = rpy2quat(-rpy);
        wrench = quatRotateVec(quat_world_to_body, [data.fx; data.fy; data.fz]);
        obj.last_wrench.setData(wrench);
      end
      
      wrench = obj.last_wrench.getData();
      
      varargout = {wrench};
      
    end
  end
  
end
