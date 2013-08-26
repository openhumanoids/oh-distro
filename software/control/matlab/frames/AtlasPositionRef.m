classdef AtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas input coordinate frame
  methods
    function obj=AtlasPositionRef(r,gains_id)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      nu = getNumInputs(r);
      dim = nu;
      
      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',dim,'x');
      obj = obj@Singleton();

      obj.nu = nu;
      input_names = r.getInputFrame().coordinates;
      input_names = regexprep(input_names,'_motor',''); % remove motor suffix
      input_frame = getInputFrame(r);
      input_frame.setCoordinateNames(input_names); % note: renaming input coordinates

      if nargin<2 % controlling robot
        warning('AtlasPositionRef: USING ATLAS GAINS')
        gains = getAtlasGains(input_frame);
      else
        warning('AtlasPositionRef: USING SIMULATION GAINS')
        typecheck(gains_id,'char');
        gains = struct();
        gains.k_qd_p = zeros(obj.nu,1);
        gains.k_q_i = zeros(obj.nu,1);
        gains.k_f_p = zeros(obj.nu,1);
        gains.ff_f_d = zeros(obj.nu,1);
        gains.ff_qd_d = zeros(obj.nu,1);
        gains.ff_const = zeros(obj.nu,1);

        [Kp,Kd] = getPDGains(r,gains_id);
        gains.k_q_p = diag(Kp);
        gains.ff_qd = diag(Kd);
      end

      coder = drc.control.AtlasCommandCoder(input_names,gains.k_q_p,gains.k_q_i,...
        gains.k_qd_p,gains.k_f_p*0,gains.ff_qd,gains.ff_qd_d*0,gains.ff_f_d*0,gains.ff_const);
      obj = setLCMCoder(obj,JLCMCoder(coder));
      
      coords = input_names;
      
      obj.setCoordinateNames(coords);
      obj.setDefaultChannel('ATLAS_COMMAND');
      
      if (obj.mex_ptr==0)
        obj.mex_ptr = AtlasCommandPublisher(input_names,gains.k_q_p,gains.k_q_i,...
        gains.k_qd_p,gains.k_f_p*0,gains.ff_qd,gains.ff_qd_d*0,gains.ff_f_d*0,gains.ff_const);
        obj = setLCMCoder(obj,JLCMCoder(coder));
      end
    end
    
    function publish(obj,t,x,channel)
      % short-cut java publish with a faster mex version
      AtlasCommandPublisher(obj.mex_ptr,channel,t,[x;0*x;0*x]);
    end
    
    function delete(obj)
      AtlasCommandPublisher(obj.mex_ptr);
    end
    
    function updateGains(obj,gains)
      assert(isfield(gains,'k_q_p'));
      assert(isfield(gains,'k_q_i'));
      assert(isfield(gains,'k_qd_p'));
      assert(isfield(gains,'k_f_p'));
      assert(isfield(gains,'ff_qd'));
      assert(isfield(gains,'ff_qd_d'));
      assert(isfield(gains,'ff_f_d'));
      assert(isfield(gains,'ff_const'));
      
      obj.mex_ptr = AtlasCommandPublisher(obj.mex_ptr,gains.k_q_p,gains.k_q_i,...
        gains.k_qd_p,gains.k_f_p*0,gains.ff_qd,gains.ff_qd_d*0,gains.ff_f_d*0,gains.ff_const);
    end

    function obj = setLCMCoder(obj,lcmcoder)
      typecheck(lcmcoder,'LCMCoder');
      obj.lcmcoder = lcmcoder;
      msg = obj.lcmcoder.encode(0,zeros(obj.nu*3,1));
      obj.monitor = drake.util.MessageMonitor(msg,obj.lcmcoder.timestampName());
      obj.lc = lcm.lcm.LCM.getSingleton();
    end  
    
  end
  
  methods (Static)
    function obj = loadobj(a)
      a.mex_ptr = 0;
      obj=a;
    end
  end

  properties
    mex_ptr=0;
    nu;
  end
end
