classdef ValkyrieInput < LCMCoordinateFrame & Singleton
  % atlas input coordinate frame
  methods
    function obj=ValkyrieInput(r)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      num_u = getNumInputs(r);
      dim = num_u;
      
      obj = obj@LCMCoordinateFrame('drcFrames.ValkyrieInput',dim,'x');
      obj = obj@Singleton();
      obj.nu=num_u;
      
      if isempty(obj.lcmcoder)  % otherwise I had a singleton
        input_names = r.getInputFrame().getCoordinateNames;
        input_names = regexprep(input_names,'_motor',''); % remove motor suffix
        input_frame = getInputFrame(r);
        input_frame.setCoordinateNames(input_names); % note: renaming input coordinates

        'todo: update gains for val'
        gains = struct();
        gains.k_f_p = ones(32, 1);
        gains.k_q_p = zeros(32, 1);
        gains.k_q_i = zeros(32, 1);
        gains.k_qd_p = zeros(32, 1);
        gains.ff_f_d = zeros(32, 1);
        gains.ff_const = zeros(32, 1);
        gains.ff_qd = zeros(32, 1);
        gains.ff_qd_d = zeros(32, 1);

        coder = drc.control.ValkyrieCommandCoder(input_names,gains.k_q_p*0,gains.k_q_i*0,...
          gains.k_qd_p*0,gains.k_f_p,gains.ff_qd,gains.ff_qd_d*0,gains.ff_f_d,gains.ff_const);
        obj = obj.setLCMCoder(JLCMCoder(coder));

        coords = input_names;

        obj.setCoordinateNames(coords);
        obj.setDefaultChannel('ROBOT_COMMAND');
      end
      
      %if (obj.mex_ptr==0)
      %  obj.mex_ptr = ValkyrieCommandPublisher(input_names,r.atlas_version,gains.k_q_p*0,gains.k_q_i*0,...
      %    gains.k_qd_p*0,gains.k_f_p,gains.ff_qd,gains.ff_qd_d*0,gains.ff_f_d,gains.ff_const);
      %end
    end
    
    function publish(obj,t,x,channel)
      % short-cut java publish with a faster mex version
      'todo'
      %ValkyrieCommandPublisher(obj.mex_ptr,channel,t,[0*x;0*x;x]);
    end
    
    function delete(obj)
      % AtlasCommandPublisher(obj.mex_ptr);
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
      
      %obj.mex_ptr = AtlasCommandPublisher(obj.mex_ptr,gains.k_q_p*0,gains.k_q_i*0,...
      %    gains.k_qd_p*0,gains.k_f_p,gains.ff_qd,gains.ff_qd_d*0,gains.ff_f_d,gains.ff_const);
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
