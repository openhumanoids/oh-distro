classdef NeckControlBlock < MIMODrakeSystem
  % passes through the robot state and
  % reads in qtraj from shared data handle and overrides neck entry
  properties
    robot;
    controller_data;
    neck_idx;
    head_idx;
    dt;
  end
  
  methods
    function obj = NeckControlBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
      
      ctrl_data = getData(controller_data);
      if ~isfield(ctrl_data,'qtraj')
        error('NeckController: controller_data must contain qtraj field');
      end
      
      if nargin<3
        options = struct();
      end
      
      neckpitch = NeckPitchFrame();
      atlas_state = getStateFrame(r);
      input_frame = MultiCoordinateFrame({neckpitch,atlas_state});
      output_frame = atlas_state;
      
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
      
      obj.robot = r;
       
      joint_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      obj.neck_idx = find(~cellfun(@isempty,strfind(joint_names,'neck')));
      obj.head_idx = findLinkInd(r,'head');
      obj = setSampleTime(obj,[obj.dt;0]); 
    end
       
    function y=mimoOutput(obj,t,~,varargin)
      neckpitch = varargin{1};
      x = varargin{2};
      q = x(1:getNumDOF(obj.robot));
      
      cdata = obj.controller_data.getData();
   
      kinsol = doKinematics(obj.robot,q,false,true);
      [~,J] = forwardKin(obj.robot,kinsol,obj.head_idx,[0;0;0],1); 
      K = 1/J(5,obj.neck_idx); % just need the sign, really

%       K = 1;
      neck_max_delta = 0.05;

      qtraj = cdata.qtraj;
      if typecheck(qtraj,'double')
        neck_des = qtraj(obj.neck_idx);
      else
        % pp trajectory
        qdes = qtraj.eval(t);
        neck_des = qdes(obj.neck_idx);
      end
      delta = min(neck_max_delta,max(-neck_max_delta,K*(neckpitch-neck_des))); % threshold to prevent jumps
      neck_des = q(obj.neck_idx) + delta;

      qtraj = cdata.qtraj;
      qtraj(obj.neck_idx) = neck_des; % THIS IS RIDICULOUSLY SLOW >>> FIX
      
      obj.controller_data.setField('qtraj',qtraj);

      y=x;
    end
  end
  
end
