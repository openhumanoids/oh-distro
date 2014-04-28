classdef FootContactBlock < MIMODrakeSystem

	properties
    contact_est_monitor;
    num_outputs;
  end
  
  methods
    function obj = FootContactBlock(r,options)
      typecheck(r,'Atlas');
            
      % num_outputs option specifies how many copies of the output to
      % return
      if isfield(options,'num_outputs')
        typecheck(options.num_outputs,'double');
        sizecheck(options.num_outputs,[1 1]);
      else
        options.num_outputs = 1;
      end
    
      input_frame = getStateFrame(r);
      [outputs{1:options.num_outputs}] = deal(FootContactState);
      output_frame = MultiCoordinateFrame(outputs);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
			
      if nargin<2
        options = struct();
      end
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate

      obj.num_outputs = options.num_outputs;
      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);      
    end
   
    function varargout=mimoOutput(obj,~,~,~)      
      % get foot contact state over LCM
      contact_data = obj.contact_est_monitor.getMessage(); % slow
      if isempty(contact_data)
        lfoot_contact_state = 0;
        rfoot_contact_state = 0;
      else
        msg = drc.foot_contact_estimate_t(contact_data);
        lfoot_contact_state = msg.left_contact > 0.5;
        rfoot_contact_state = msg.right_contact > 0.5;
      end
      
			y = [lfoot_contact_state; rfoot_contact_state]      
      if obj.num_outputs > 1
        varargout = cell(1,obj.num_outputs);
        for i=1:obj.num_outputs
          % looping seems faster than using deal or arrayfun
          varargout{i} = y;
        end
      else
        varargout = y;
      end
		end
  end
  
end
