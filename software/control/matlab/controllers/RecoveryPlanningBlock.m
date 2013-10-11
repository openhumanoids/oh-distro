classdef RecoveryPlanningBlock < DrakeSystem
  properties
    dt;
    controller_data; % pointer to shared data handle containing foot trajectories
    robot;
    nq;
    contact_state;
    lcmgl
  end
  
  methods
    function obj = RecoveryPlanningBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      typecheck(controller_data,'SharedDataHandle');
            
      input_frame = getStateFrame(r);
      output_frame = getStateFrame(r);
      obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);

      obj.controller_data = controller_data;
      obj.nq = getNumDOF(r);
      obj.lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'recovery_planner');

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
      obj.robot = r;
    end
    
    function y=output(obj,t,~,x)
      t
      period = 0.5;
      persistent has_run
      persistent footsteps
      if isempty(has_run) || (mod(t, period) > 0.1 && mod(t, period) < 0.2 && ~has_run)
      % if isempty(has_run)
        has_run = true;
        q = x(1:obj.nq);
        contact_threshold = 0.001; % a point is considered to be in contact if within this distance
        kinsol = doKinematics(obj.robot,q,false,true);
        ctrl_data = obj.controller_data.data;
        obj.controller_data.setField('trans_drift',[0;0;0]);
        
        supp_idx = find(ctrl_data.support_times<=t,1,'last');
        supp = ctrl_data.supports(supp_idx);
        i=1;
        if isempty(footsteps)
          obj.contact_state.right_contact = false;
          obj.contact_state.left_contact = false;
          while i<=length(supp.bodies)
            % check kinematic contact
            phi = contactConstraints(obj.robot,kinsol,supp.bodies(i),supp.contact_pts{i});
            contact_state_kin = any(phi<=contact_threshold);
            if supp.bodies(i) == obj.robot.foot_bodies_idx(1)
              obj.contact_state.right_contact = contact_state_kin;
            elseif supp.bodies(i) == obj.robot.foot_bodies_idx(2)
              obj.contact_state.left_contact = contact_state_kin;
            else
              error('bad supp body')
            end
            i=i+1;
          end
        else
          if footsteps(1).is_right_foot
            obj.contact_state.right_contact = true;
            obj.contact_state.left_contact = false;
          else
            obj.contact_state.right_contact = false;
            obj.contact_state.left_contact = true;
          end
        end
        recovery_opts = struct('dts', [period-mod(t, period), period]);
        % recovery_opts = struct('dts', [period, period]);
        [footsteps,result] = recoverySteps(obj.robot, x, obj.contact_state, recovery_opts);
        if isfield(result, 'x')
          for j = 1:length(footsteps)
            if footsteps(j).is_right_foot
              obj.lcmgl.glColor3f(0,1,0);
            else
              obj.lcmgl.glColor3f(1,0,0);
            end
            obj.lcmgl.sphere(footsteps(j).pos(1:3),0.02,20,20);
            if j > 2
              pos = fitStepToTerrain(obj.robot, footsteps(j).pos, 'center');
            else
              pos = footsteps(j).pos;
            end
            footsteps(j).pos = obj.robot.footContact2Orig(pos, 'center', footsteps(j).is_right_foot);
          end
          obj.lcmgl.switchBuffers();
          footstep_opts = struct('ignore_terrain', 1, 'mu', 1, 'behavior', drc.footstep_opts_t.BEHAVIOR_WALKING,'t0',t);
          [support_times, supports, comtraj, foottraj, V, zmptraj] = walkingPlanFromSteps(obj.robot, x, footsteps, footstep_opts);
          s1dot = fnder(V.s1,1);
          s2dot = fnder(V.s2,1);
          ts = 0:0.1:zmptraj.tspan(end);
          T = ts(end);
          link_constraints = buildLinkConstraints(obj.robot, q, foottraj, []);
          obj.controller_data.setField('S', V.S.eval(0));
          obj.controller_data.setField('s1', V.s1);
          obj.controller_data.setField('s2', V.s2);
          obj.controller_data.setField('s1dot', s1dot);
          obj.controller_data.setField('s2dot', s2dot);
          obj.controller_data.setField('x0', [zmptraj.eval(T);0;0]);
          obj.controller_data.setField('comtraj', comtraj);
          obj.controller_data.setField('link_constraints', link_constraints);
          obj.controller_data.setField('support_times', support_times);
          obj.controller_data.setField('supports', [supports{:}]);
          obj.controller_data.setField('y0', zmptraj);
        end
      elseif mod(t, period) > 0.2
        has_run = false;
      end
      y=x;
    end
  end
  
end
