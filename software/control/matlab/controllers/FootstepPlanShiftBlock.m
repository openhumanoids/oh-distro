classdef FootstepPlanShiftBlock < MIMODrakeSystem
  properties
    dt;
    controller_data; % pointer to shared data handle containing foot trajectories
    robot;
    nq;
    lfoot_ind;
    rfoot_ind;
    use_mex;
    mex_ptr;
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
      obj.nq = getNumPositions(r);

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

      if isfield(options,'use_mex')
        sizecheck(options.use_mex,1);
        obj.use_mex = uint32(options.use_mex);
        rangecheck(obj.use_mex,0,2);
        if (obj.use_mex && exist('footstepPlanShiftmex','file')~=3)
          error('can''t find footstepPlanShiftmex.  did you build it?');
        end
      else
        obj.use_mex = 1;
      end

      obj.robot = r;
      if (obj.use_mex>0)
        obj.mex_ptr = SharedDataHandle(footstepPlanShiftmex(0,obj.robot.getMexModelPtr.ptr,1.0/obj.dt));
      end

      obj.rfoot_ind = findLinkInd(r,'r_foot');
      obj.lfoot_ind = findLinkInd(r,'l_foot');
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
        loading_foot = obj.rfoot_ind; % arbitrarily pick the right foot
      else
        loading_foot = supp.bodies;
      end 

      ctrl_data = obj.controller_data;
      lfoot_link_con_ind = [ctrl_data.link_constraints.link_ndx]==obj.lfoot_ind;
      rfoot_link_con_ind = [ctrl_data.link_constraints.link_ndx]==obj.rfoot_ind;
      % ts should be identical for left/right foot
      foot_traj_ind = find(ctrl_data.link_constraints(lfoot_link_con_ind).ts<=t,1,'last');
      tt = t-ctrl_data.link_constraints(lfoot_link_con_ind).ts(foot_traj_ind);
      
      a0 = ctrl_data.link_constraints(lfoot_link_con_ind).a0(:,foot_traj_ind);
      a1 = ctrl_data.link_constraints(lfoot_link_con_ind).a1(:,foot_traj_ind);
      a2 = ctrl_data.link_constraints(lfoot_link_con_ind).a2(:,foot_traj_ind);
      a3 = ctrl_data.link_constraints(lfoot_link_con_ind).a3(:,foot_traj_ind);
      lfoot_des = evalCubicSplineSegment(tt,a0, a1,a2,a3);

      a0 = ctrl_data.link_constraints(rfoot_link_con_ind).a0(:,foot_traj_ind);
      a1 = ctrl_data.link_constraints(rfoot_link_con_ind).a1(:,foot_traj_ind);
      a2 = ctrl_data.link_constraints(rfoot_link_con_ind).a2(:,foot_traj_ind);
      a3 = ctrl_data.link_constraints(rfoot_link_con_ind).a3(:,foot_traj_ind);
      rfoot_des = evalCubicSplineSegment(tt,a0,a1,a2,a3);
      if (obj.use_mex == 0)
        persistent last_t;
        if (isempty(last_t) || last_t > t)
          last_t = 0;
        end
        if (t - last_t >= obj.dt)
          last_t = t;
          cdata = obj.controller_data;
          if fc(1) > 0.5 && loading_foot==obj.lfoot_ind % left foot in contact
            kinsol = doKinematics(obj.robot,x(1:obj.nq),false,true);
            lfoot_act = forwardKin(obj.robot,kinsol,obj.lfoot_ind,[0;0;0],0);
            plan_shift = lfoot_des(1:3) - lfoot_act(1:3);
            % fprintf('LF:Footstep desired minus actual: x:%2.4f y:%2.4f z:%2.4f m \n',plan_shift);
            obj.controller_data.plan_shift = plan_shift;
          elseif fc(2) > 0.5 && loading_foot==obj.rfoot_ind % right foot in contact
            kinsol = doKinematics(obj.robot,x(1:obj.nq),false,true);
            rfoot_act = forwardKin(obj.robot,kinsol,obj.rfoot_ind,[0;0;0],0);
            plan_shift = rfoot_des(1:3) - rfoot_act(1:3);
            % fprintf('RF:Footstep desired minus actual: x:%2.4f y:%2.4f z:%2.4f m \n',plan_shift);
            obj.controller_data.plan_shift = plan_shift;
          end
        end
      else
        obj.controller_data.plan_shift = footstepPlanShiftmex(obj.mex_ptr.data,t,x,fc(1),fc(2),lfoot_des(1:3),rfoot_des(1:3),ctrl_data.plan_shift);  
      end
      y=x;
    end
  end  
end
