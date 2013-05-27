classdef NeckControlBlock < MIMODrakeSystem
  % passes through the robot state and
  % reads in qtraj from shared data handle and overrides neck entry
  properties
    robot;
    controller_data;
    neck_idx;
    head_idx;
    dt;
    nq;
  end
  
  methods
    function obj = NeckControlBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      
      ctrl_data = getData(controller_data);
      if ~isfield(ctrl_data,'qtraj')
        error('NeckControlBlock: controller_data must contain qtraj field');
      end
      
      if nargin<3
        options = struct();
      end
      
      atlas_state = getStateFrame(r);
      input_frame = MultiCoordinateFrame({NeckPitchFrame(),atlas_state});
      output_frame = MultiCoordinateFrame({AtlasCoordinates(r),atlas_state});
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.robot = r;
      obj.controller_data = controller_data;
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.005;
      end
      
      joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      obj.nq=getNumDOF(obj.robot);
      obj.neck_idx = find(~cellfun(@isempty,strfind(joint_names,'neck')));
      obj.head_idx = findLinkInd(r,'head');
      obj = setSampleTime(obj,[obj.dt;0]); 
    end
       
    function [qdes,x]=mimoOutput(obj,t,~,varargin)
      neckpitch = varargin{1};
      x = varargin{2};
      q = x(1:obj.nq);
      qd = x(obj.nq+(1:obj.nq));

      Kp = 60;
      Kd = 5;
      neck_max_delta = 0.075;

      % OPT: all of these interactions with PPTrajectories are expensive
      qtraj = obj.controller_data.getField('qtraj');
      if typecheck(qtraj,'double')
        qdes=qtraj;
      else
        % pp trajectory
        qdes = qtraj.eval(t);
      end
      
      delta = Kp*(neckpitch-q(obj.neck_idx)) - Kd*qd(obj.neck_idx);
      delta = min(neck_max_delta,max(-neck_max_delta,delta)); % threshold to prevent jumps
      qdes(obj.neck_idx) = q(obj.neck_idx) + delta; 
      
    end
  end
  
end
