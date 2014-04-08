classdef RecoveryPlanningBlock < DrakeSystem
  properties
    dt;
    controller_data; % pointer to shared data handle containing foot trajectories
    robot;
    nq;
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
      persistent last_plan_t
      persistent contact_state
      persistent footsteps
      if isempty(last_plan_t); last_plan_t = -inf; end
      if isempty(contact_state)
        contact_state = struct('right_contact', false, 'left_contact', false);
      end
      contact_threshold = 0.001; % a point is considered to be in contact if within this distance
      % obj.controller_data.setField('trans_drift',[0;0;0]);
      period = 0.6;
      ctrl_data = obj.controller_data.data;
      q = x(1:obj.nq);
      kinsol = doKinematics(obj.robot,q,false,true);
      supp_idx = find(ctrl_data.support_times<=t,1,'last');
      supp = ctrl_data.supports(supp_idx);
      i=1;
      cstate.right_contact = false;
      cstate.left_contact = false;
      while i<=length(supp.bodies)
        % check kinematic contact
        phi = contactConstraints(obj.robot,kinsol,supp.bodies(i),supp.contact_pts{i});
        contact_state_kin = any(phi<=contact_threshold);
        if supp.bodies(i) == obj.robot.foot_bodies_idx.right
          cstate.right_contact = contact_state_kin;
        elseif supp.bodies(i) == obj.robot.foot_bodies_idx.left
          cstate.left_contact = contact_state_kin;
        else
          error('bad supp body')
        end
        i=i+1;
      end

      halfway = t - last_plan_t > period / 2;
      is_switching = isempty(footsteps) || (halfway && ((footsteps(1).is_right_foot && cstate.right_contact > contact_state.right_contact) ||...
                      (~footsteps(1).is_right_foot && cstate.left_contact > contact_state.left_contact)));
      if is_switching
      % if 1

        contact_state = cstate;

        if is_switching
          last_plan_t = t;
          dts = [period, period];
          if ~isempty(footsteps)
            if footsteps(1).is_right_foot
              cstate.right_contact = true;
              cstate.left_contact = false;
            else
              cstate.right_contact = false;
              cstate.left_contact = true;
            end
          end
        else
          dts = [period - (t - last_plan_t), period]
        end
        recovery_opts = struct('dts', dts,'n_steps',2);
        [footsteps,result] = recoverySteps(obj.robot, x, cstate, recovery_opts);
        if isfield(result, 'x')
          for j = 1:size(result.footxy,2)
            obj.lcmgl.glColor3f(0.7,0.7,0.7);
            obj.lcmgl.sphere([result.footxy(:,j)',0],0.02,20,20);
            obj.lcmgl.glColor3f(1,0.6,0.6);
            obj.lcmgl.sphere([result.COP(:,j)',0],0.02,20,20);
          end
          obj.lcmgl.glColor3f(1.0,0.2,0.2);
          obj.lcmgl.glLineWidth(1);
          obj.lcmgl.glBegin(obj.lcmgl.LCMGL_LINES);
          for j = 1:size(result.IC,2)-1
            obj.lcmgl.glVertex3f(result.IC(1,j), result.IC(2,j), 0);
            obj.lcmgl.glVertex3f(result.IC(1,j+1), result.IC(2,j+1), 0);
          end
          obj.lcmgl.glEnd();

          for j = 1:length(footsteps)
            % footsteps(j).step_speed = 1.5;
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
      else
        % last_plan_t = t;
        contact_state = cstate;
      end
      y=x;
    end
  end

end
