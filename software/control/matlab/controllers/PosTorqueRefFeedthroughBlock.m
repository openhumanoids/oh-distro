classdef PosTorqueRefFeedthroughBlock < MIMODrakeSystem
  % combines joint position and torque references and sends to atlas 
  
  properties
    act_idx;
    robot;
  end
  
  methods
    function obj = PosTorqueRefFeedthroughBlock(r,options)
      typecheck(r,'Atlas');
  
      if nargin<2
        options = struct();
      else
        typecheck(options,'struct');
      end
 
      robot_input_frame = getInputFrame(r);
      input_frame = MultiCoordinateFrame({AtlasCoordinates(r),robot_input_frame});
      output_frame = AtlasPosTorqueRef(r);
         
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      % TEMP: only use force control for arms
      gains = getAtlasGains(robot_input_frame);
      arm_joints = ~cellfun(@isempty,strfind(robot_input_frame.coordinates,'r_arm'));
      gains.k_f_p(~arm_joints) = zeros(sum(~arm_joints),1);
      gains.ff_f_d(~arm_joints) = zeros(sum(~arm_joints),1);
      gains.ff_qd(~arm_joints) = zeros(sum(~arm_joints),1);
      % use relatively weak P gains for arms to pick up slack from model
      % errors
      gains.k_q_p(arm_joints) = 0*4*ones(sum(arm_joints),1);
      gains.k_qd_p(arm_joints) = 0*0.1*ones(sum(arm_joints),1);
 
      output_frame.updateGains(gains);
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.003;
      end
      
      obj.robot = r;
      obj.act_idx = getActuatedJoints(r);
      
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      q_des = varargin{1}; % in state frame ordering
      u_des = varargin{2}; % in input frame ordering
      y = [q_des(obj.act_idx); u_des];
    end
  end
  
end
