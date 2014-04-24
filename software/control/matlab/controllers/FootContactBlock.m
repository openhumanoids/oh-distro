classdef FootContactBlock < DrakeSystem

	properties
    contact_est_monitor;
  end
  
  methods
    function obj = FootContactBlock(r,options)
      typecheck(r,'Atlas');
            
      input_frame = getStateFrame(r);
      output_frame = FootContactState;
		
			obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,false);
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

      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      lc = lcm.lcm.LCM.getSingleton();
      lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);      
    end
   
    function y=output(obj,t,~,x)      
      % get foot contact state over LCM
      contact_data = obj.contact_est_monitor.getMessage();
			if isempty(contact_data)
        lfoot_contact_state = 0;
        rfoot_contact_state = 0;
      else
        msg = drc.foot_contact_estimate_t(contact_data);
        lfoot_contact_state = msg.left_contact > 0.5;
        rfoot_contact_state = msg.right_contact > 0.5;
			end

			y = [lfoot_contact_state; rfoot_contact_state];      
		end
  end
  
end
