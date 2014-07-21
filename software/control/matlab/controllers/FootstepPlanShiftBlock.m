classdef FootstepPlanShiftBlock < MIMODrakeSystem
  properties
    dt;
    controller_data; % pointer to shared data handle containing foot trajectories
    robot;
    nq;
    rfoot_idx;
    lfoot_idx;
  end
  
  methods
    function obj = FootstepPlanShiftBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'QPControllerData');
            
      input_frame = MultiCoordinateFrame({getStateFrame(r),FootContactState});
      output_frame = getStateFrame(r);
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
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
        obj.dt = 0.1;
      end
%       obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate

      obj.rfoot_idx = findLinkInd(r,'r_foot');
      obj.lfoot_idx = findLinkInd(r,'l_foot');
      obj.robot = r;
    end
    
    function y=mimoOutput(obj,t,~,varargin)
      x = varargin{1};
      fc = varargin{2};
    
      ctrl_data = obj.controller_data;

      supp_idx = find(ctrl_data.support_times<=t,1,'last');
      supp = ctrl_data.supports(supp_idx);      
      while length(supp.bodies) > 1 && supp_idx < length(ctrl_data.support_times)
        supp_idx = supp_idx + 1;
        supp = ctrl_data.supports(supp_idx);      
      end
      if length(supp.bodies)==2
        loading_foot = obj.rfoot_idx; % arbitrarily pick the right foot
      else
        loading_foot = supp.bodies;
      end 
      
      persistent last_t;
      if (isempty(last_t) || last_t > t)
        last_t = 0;
      end
      if (t - last_t >= obj.dt)
        last_t = t;
        cdata = obj.controller_data;
        if fc(1) > 0.5 && loading_foot==obj.lfoot_idx % left foot in contact
          q = x(1:obj.nq); 
          kinsol = doKinematics(obj.robot,q,false,true);

          constraint_ndx = [cdata.link_constraints.link_ndx] == obj.lfoot_idx & all(bsxfun(@eq, [cdata.link_constraints.pt], [0;0;0]));
          lfoot_des = fasteval(cdata.link_constraints(constraint_ndx).traj,t);
          lfoot_act = forwardKin(obj.robot,kinsol,obj.lfoot_idx,[0;0;0],0);
          plan_shift = lfoot_des(1:3) - lfoot_act(1:3);
          % fprintf('LF:Footstep desired minus actual: x:%2.4f y:%2.4f z:%2.4f m \n',plan_shift);
          obj.controller_data.plan_shift = plan_shift;

        elseif fc(2) > 0.5 && loading_foot==obj.rfoot_idx % right foot in contact
          q = x(1:obj.nq); 
          kinsol = doKinematics(obj.robot,q,false,true);

          constraint_ndx = [cdata.link_constraints.link_ndx] == obj.rfoot_idx & all(bsxfun(@eq, [cdata.link_constraints.pt], [0;0;0]));
          rfoot_des = fasteval(cdata.link_constraints(constraint_ndx).traj,t);
          rfoot_act = forwardKin(obj.robot,kinsol,obj.rfoot_idx,[0;0;0],0);
          plan_shift = rfoot_des(1:3) - rfoot_act(1:3);

          % fprintf('RF:Footstep desired minus actual: x:%2.4f y:%2.4f z:%2.4f m \n',plan_shift);
          obj.controller_data.plan_shift = plan_shift;
        end
      end
      y=x;
    end
  end  
end
