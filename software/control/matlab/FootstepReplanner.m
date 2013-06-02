classdef FootstepReplanner < DrakeSystem
  % outputs a desired q_ddot (including floating dofs)
  properties
    dt;
    controller_data; % pointer to shared data handle containing foot trajectories
    robot;
    nq;
    rfoot_idx;
    lfoot_idx;
    lc;
    contact_est_monitor;
    lfoot_contact_state = 0; 
    rfoot_contact_state = 0; 
  end
  
  methods
    function obj = FootstepReplanner(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
            
      input_frame = getStateFrame(r);
      output_frame = getStateFrame(r);
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.controller_data = controller_data;
      obj.nq = getNumDOF(r);

      if nargin<3
        options = struct();
      end
        
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.2;
      end
      
%       obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate

      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.rfoot_idx = findLinkInd(r,'r_foot');
      obj.lfoot_idx = findLinkInd(r,'l_foot');

      obj.contact_est_monitor = drake.util.MessageMonitor(drc.foot_contact_estimate_t,'utime');
      obj.lc.subscribe('FOOT_CONTACT_ESTIMATE',obj.contact_est_monitor);
      
      obj.robot = r;
    end
    
    function y=output(obj,t,~,x)
      persistent lfoot_contact_state rfoot_contact_state;
        
      if mod(t,obj.dt)==0 % manually enforce sample time here, not the correct way to do it because states do not arrive at a reliable interval
        if isempty(lfoot_contact_state)
          lfoot_contact_state = false;
        end
        if isempty(rfoot_contact_state)
          rfoot_contact_state = false;
        end
        contact_data = obj.contact_est_monitor.getNextMessage(0);
        if ~isempty(contact_data)
          msg = drc.foot_contact_estimate_t(contact_data);

          if msg.left_contact>0.5 && ~lfoot_contact_state
            % left foot coming into contact
            cdata = obj.controller_data.getData();
            q = x(1:obj.nq); 
            kinsol = doKinematics(obj.robot,q,false,true);

            constraint_ndx = [cdata.link_constraints.link_ndx] == obj.lfoot_idx & all(bsxfun(@eq, [cdata.link_constraints.pt], [0;0;0]));
            lfoot_des = cdata.link_constraints(constraint_ndx).traj.eval(t);
            lfoot_act = forwardKin(obj.robot,kinsol,obj.lfoot_idx,[0;0;0],0);
            diffz = lfoot_act(3) - lfoot_des(3);
            cdata.z_drift = -diffz;
            
            fprintf('LF:Adjusting footsteps by %2.4f m \n',diffz);
            obj.controller_data.setField('z_drift', cdata.z_drift);
          elseif msg.right_contact>0.5 && ~rfoot_contact_state
            % right foot coming into contact
            cdata = obj.controller_data.getData();
            q = x(1:obj.nq); 
            kinsol = doKinematics(obj.robot,q,false,true);

            constraint_ndx = [cdata.link_constraints.link_ndx] == obj.rfoot_idx & all(bsxfun(@eq, [cdata.link_constraints.pt], [0;0;0]));
            rfoot_des = cdata.link_constraints(constraint_ndx).traj.eval(t);
            rfoot_act = forwardKin(obj.robot,kinsol,obj.rfoot_idx,[0;0;0],0);
            diffz = rfoot_act(3) - rfoot_des(3);
            cdata.z_drift = -diffz;

            fprintf('RF:Adjusting footsteps by %2.4f m \n',diffz);
            obj.controller_data.setField('z_drift', cdata.z_drift);
          end

          lfoot_contact_state = msg.left_contact>0.5;
          rfoot_contact_state = msg.right_contact>0.5;
        end
      end
      y=x;
    end
  end
  
end
