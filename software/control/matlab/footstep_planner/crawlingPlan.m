function [qdtraj,support_times,supports,V,comtraj,zmptraj,link_constraints] = crawlingPlan(r,x0,body_spec,foot_spec,options)
%
% @param r the robot 
% @param x0 initial state
% @param body_spec struct with body_spec.body_ind and body_spec.pt for the
%        "center" of the robot
% @param foot_spec 4 element struct with footspec(i).body_ind
%                                       footspec(i).contact_pt_ind  
%
%
% @option num_steps will be rounded up to be a multiple of 4
% @option step_speed in m/s
% @option step_height in m
% @option step_length in m
% @option com_height in m
% @option front_right_foot  xy vector from body to nominal front right foot 
% @option ignore_terrain
% @options direction - 0 for forward, <0 for left, >0 for right 
% @options gait - 0 for quasi-static walk, 1 for zmp-walk, 2 for zmp-trot
% @options duty_factor fraction of total stride time that each foot is in stance
addpath(fullfile(getDrakePath,'examples','ZMP'));

typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
sizecheck(x0,[getNumStates(r) 1]);
nq = getNumDOF(r);

fieldcheck(body_spec,'body_ind');
fieldcheck(body_spec,'pt');
sizecheck(body_spec.pt,[3 1]);

sizecheck(foot_spec,4);
fieldcheck(foot_spec,'body_ind');
fieldcheck(foot_spec,'contact_pt_ind');
for i=1:4, 
  pts = getContactPoints(getLink(r,foot_spec(i).body_ind));
  foot_spec(i).contact_pt_ind = foot_spec(i).contact_pt_ind(1); 
  foot_spec(i).contact_pt = pts(:,foot_spec(i).contact_pt_ind);
end


if nargin<3 || isempty(options), options=struct(); end
if ~isfield(options,'num_steps') options.num_steps = 20; end
if ~isfield(options,'duty_factor') options.duty_factor = 2/3; end
if ~isfield(options,'step_length') options.step_length = .3; end
if ~isfield(options,'step_speed') options.step_speed = .5; end  
if ~isfield(options,'step_height') options.step_height = .2; end
if ~isfield(options,'com_height') options.com_height = .35; end
if ~isfield(options,'comfortable_footpos') options.comfortable_footpos = [-.7 -.7 .6 .6; .3 -.3 -.3 .3]; end
if ~isfield(options,'ignore_terrain') options.ignore_terrain = true; end  % todo: make this default to false
if ~isfield(options,'direction') options.direction = 0; end
if ~isfield(options,'gait') options.gait = 2; end
if ~isfield(options,'draw') options.draw = true; end
if ~isfield(options,'debug') options.debug = false; end
if ~isfield(options,'x_nom') options.x_nom = x0; end
if ~isfield(options,'delta_yaw') options.delta_yaw = 10*pi/180; end
delta_yaw = options.direction*options.delta_yaw;
q_nom = options.x_nom(1:nq);

% always take 4 steps at a time
options.num_strides = ceil(options.num_steps/4);
options.num_steps = 4*options.num_strides;
swing_duration = abs(options.step_length/options.step_speed);
stride_duration = swing_duration/(1-options.duty_factor);
stance_duration = stride_duration*options.duty_factor;
actuated = getActuatedJoints(r);

persistent mex_ptr;

if options.draw
  %v = r.constructVisualizer();

  if isempty(mex_ptr)
    input_names = r.getInputFrame().coordinates;
    input_names = regexprep(input_names,'_motor',''); % remove motor suffix
    [Kp,Kd] = getPDGains(r,'crawling');
    
    mex_ptr = AtlasCommandPublisher(input_names,diag(Kp),diag(Kd));
  end
end

  cost = Point(getStateFrame(r),1);
  cost.base_x = 0;
  cost.base_y = 0;
  cost.base_z = 0;
  cost.base_roll = 100;
  cost.base_pitch = 10;
  cost.base_yaw = 0;
  cost.back_lbz = 10;
  cost.back_mby = 100;
  cost.back_ubx = 100;
  cost = double(cost);
  options.Q = diag(cost(1:nq));
  options.q_nom = q_nom;

  function q = crawlIK(q0,com,fpos,swing_legs)
    if nargin<4, swing_legs=[]; end
    stance_legs = 1:4; stance_legs(swing_legs)=[];

    args = {0, com}; % com
    for i=stance_legs
      args = horzcat(args,{foot_spec(i).body_ind,foot_spec(i).contact_pt,fpos(:,i)});
    end
    for i=swing_legs(:)'
      if (fpos(2,i)>0) ymin = fpos(2,i); else ymin = -inf; end
      if (fpos(2,i)<0) ymax = fpos(2,i); else ymax = inf; end
      p.min = [fpos(1,i);ymin;fpos(3,i)];
      p.max = [fpos(1,i);ymax;nan];
      args = horzcat(args,{foot_spec(i).body_ind,foot_spec(i).contact_pt,p});
    end

    q = inverseKin(r,q0,args{:},options);
    
  end

  function display(q,com,fpos,swing_legs)
    if options.draw
      error('No display function!');
      stance_legs = 1:4; stance_legs(swing_legs)=[];
      kinsol = doKinematics(r,q);
      com_real = getCOM(r,kinsol);
      pelvis_origin = forwardKin(r,kinsol,body_spec.body_ind,body_spec.pt);
      sfigure(1);hold on; axis([-2,2,-2,2]); axis equal; grid on;
      plot([fpos(1,stance_legs),fpos(1,stance_legs(1))],[fpos(2,stance_legs),fpos(2,stance_legs(1))],'go--',...
        fpos(1,swing_legs),fpos(2,swing_legs),'go',...
        com(1),com(2),'rx',...
        com_real(1),com_real(2),'bo',...
        pelvis_origin(1),pelvis_origin(2),'ms',...
        'MarkerSize',4);
      v.draw(0,[q;0*q]);
      AtlasCommandPublisher(mex_ptr,'ATLAS_COMMAND',0,q(actuated));
      pause(2);
    end
  end

% ActionSequence crawl;
crawl_sequence = ActionSequence();
link_constraints = struct('link_ndx', {}, 'pt', {}, 'min_traj', [], 'max_traj', [], 'traj', {});

% Determine forward crawling direction
z_proj  = rpy2rotmat(x0(4:6))*[0;0;1];
up_dir = [0;0;1];
forward_dir = [z_proj(1:2);0];
forward_dir = forward_dir/norm(forward_dir);
left_dir = cross(up_dir,forward_dir);

% Determine nominal forward crawling direction
z_proj_nom = rpy2rotmat(options.x_nom(4:6))*[0;0;1];
forward_dir_nom = [z_proj_nom(1:2);0];
forward_dir_nom = forward_dir_nom/norm(forward_dir_nom);
left_dir_nom = rotz(pi/2)*forward_dir_nom;

kinsol = doKinematics(r,q_nom);
for i = 1:4
  fpos_nom(:,i) = forwardKin(r,kinsol,foot_spec(i).body_ind,foot_spec(i).contact_pt);
  fpos_rel_nom(:,i) = fpos_nom(:,i) - q_nom(1:3);
  forward_coord_nom = fpos_rel_nom'*forward_dir_nom;
  left_coord_nom = fpos_rel_nom'*left_dir_nom;
end

q = x0(1:nq);
kinsol = doKinematics(r,q);
for i=1:4
  fpos_initial(:,i) = forwardKin(r,kinsol,foot_spec(i).body_ind,foot_spec(i).contact_pt);
end
com_initial = getCOM(r,kinsol);
z_foot_nom = mean(fpos_initial(3,:));
pelvis_ind = r.findLinkInd('pelvis');
pelvis_pos_initial = forwardKin(r,kinsol,pelvis_ind,[0;0;0]);
%hip0 = forwardKin(r,kinsol,body_spec.body_ind,body_spec.pt);
%display(q,com,fpos,[]); pause(5);

if (options.gait==0) % quasi-static
  error('quasi-static walk not presently supported');
  %order = [4 2 3 1];
  %for step=1:4:options.num_steps
    %for swing_leg=order
      %stance_legs = [1:swing_leg-1,swing_leg+1:4];
      
      %% todo: add constraints to crawl sequence

      %% prepare for step (move COM to support triangle)
      %a = mean(fpos(1:2,stance_legs),2);
      %b = fpos(1:2,swing_leg);
      %com = [.9*a+.1*b;options.com_height];
      %q = crawlIK(q,com,fpos);
      %display(q,com,fpos,swing_leg);
      
      %% apex of swing leg
      %if (step==1 || step==options.num_steps)
        %fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/4;
      %else
        %fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/2;
      %end
      %fpos(3,swing_leg)=options.step_height;
      %q = crawlIK(q,com,fpos,swing_leg);
      %display(q,com,fpos,swing_leg);
      
      %% end of step
      %if (step==1 || step==options.num_steps)
        %fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/4;
      %else
        %fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/2;
      %end
      %fpos(3,swing_leg)=0;
      %q = crawlIK(q,com,fpos);
      %display(q,com,fpos,swing_leg);
      %step=step+1;
    %end
  %end
elseif (options.gait ==2) % trot
  %t_start = stride_duration/2 - swing_duration;
  t_start = stride_duration;

  fpos = zeros(3,4);
  fpos_rel = bsxfun(@times,forward_dir,forward_coord_nom') + bsxfun(@times,left_dir,left_coord_nom');
  fpos_start = bsxfun(@plus,x0(1:3),fpos_rel);
  %fpos_start(:,1) = x0(1:3) + forward_dir*forward_coord_nom(1) + left_dir*left_coord_nom(1);
  %fpos_start(:,2) = x0(1:3) + forward_dir*forward_coord_nom(2) + left_dir*left_coord_nom(2);
  %fpos_start(:,3) = x0(1:3) + forward_dir*forward_coord_nom(3) + left_dir*left_coord_nom(3);
  %fpos_start(:,4) = x0(1:3) + forward_dir*forward_coord_nom(4) + left_dir*left_coord_nom(4);
  fpos_start(3,:) = z_foot_nom;
  fpos_all{1} = fpos_start;
  for i = 2:options.num_strides+1
    if options.direction ~= 0
      fpos_all{i} = rotz(options.delta_yaw)*fpos_all{i-1};
    else
      fpos_all{i} = bsxfun(@plus,options.step_length*forward_dir,fpos_all{i-1});
    end
  end
  com_start = [mean(fpos_start(1:2,:),2);options.com_height+z_foot_nom];

  for i=1:4
    body = getLink(r,foot_spec(i).body_ind);
    body_pts = body.contact_pts(:,foot_spec(i).contact_pt_ind);
    if any(i == [1,3])
      fpos = fpos_start;
      % Initial stance constraints
      %tspan = [t_start/2,t_start];
      tspan = [0,t_start];
      kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,fpos(:,i), tspan,'', ...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
        ActionKinematicConstraint.BREAK_CONTACT);
      crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
      if options.draw
        sfigure(7); hold on; grid on;
        plot(tspan,[i i],'s-');
        sfigure(8); hold on; grid on;
        plot(fpos(1,i),fpos(2,i),'bo');
      end
      if options.direction == 0
        fpos(:,i) = fpos(:,i) + forward_dir* options.step_length/2;
      else
        fpos(:,i) = fpos_all{2}(:,i);
      end

      % Intermediate stance constraints
      for j = 1:options.num_strides-1
        tspan = t_start + (j-1)*stride_duration + [swing_duration,stride_duration];
        kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,fpos(:,i), tspan,'',...
          ActionKinematicConstraint.MAKE_CONTACT, ...
          ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
          ActionKinematicConstraint.BREAK_CONTACT);
        crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
        if options.draw
          sfigure(7); hold on; grid on;
          plot(tspan,[i i],'s-');
          sfigure(8); hold on; grid on;
          plot(fpos(1,i),fpos(2,i),'bo');
        end
        if options.direction == 0
          fpos(:,i) = fpos(:,i) + forward_dir*options.step_length;
        else
          fpos(:,i) = fpos_all{j+2}(:,i);
        end
      end
      % Final stance constraints
      j = options.num_strides;
      tspan = t_start + (j-1)*stride_duration + [swing_duration,stride_duration];
      kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,fpos(:,i), tspan,'',...
        ActionKinematicConstraint.MAKE_CONTACT, ...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT);
      crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
      if options.draw
        sfigure(7); hold on; grid on;
        plot(tspan,[i i],'s-');
        sfigure(8); hold on; grid on;
        plot(fpos(1,i),fpos(2,i),'bo');
      end

      % Apex constraints
      fpos = fpos_start;
      j = 1;
      tspan = t_start + (j-1)*stride_duration + [0,swing_duration];
      x0_foot = fpos(:,i);
      if options.direction == 0
        fpos(:,i) = fpos(:,i) + forward_dir* options.step_length/2;
      else
        fpos(:,i) = fpos_all{2}(:,i);
      end
      xf_foot = fpos(:,i);
      f_spline = PPTrajectory(swing_foot_spline(x0_foot,xf_foot,options.step_height,tspan(2)-tspan(1),tspan(1)));
      link_constraints(end+1) = struct('link_ndx', foot_spec(i).body_ind, 'pt', body.contact_pts(:,foot_spec(i).contact_pt_ind), 'min_traj', f_spline, 'max_traj', f_spline, 'traj', f_spline);
      kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,f_spline,tspan,'', ...
        ActionKinematicConstraint.NOT_IN_CONTACT, ...
        ActionKinematicConstraint.NOT_IN_CONTACT, ...
        ActionKinematicConstraint.NOT_IN_CONTACT);
      crawl_sequence = addKinematicConstraint(crawl_sequence,kc);

      for j = 2:options.num_strides
        tspan = t_start + (j-1)*stride_duration + [0,swing_duration];
        x0_foot = fpos(:,i);
        if options.direction == 0
          fpos(:,i) = fpos(:,i) + forward_dir*options.step_length;
        else
          fpos(:,i) = fpos_all{j+1}(:,i);
        end
        xf_foot = fpos(:,i);
        f_spline = PPTrajectory(swing_foot_spline(x0_foot,xf_foot,options.step_height,tspan(2)-tspan(1),tspan(1)));
        link_constraints(end+1) = struct('link_ndx', foot_spec(i).body_ind, 'pt', body.contact_pts(:,foot_spec(i).contact_pt_ind), 'min_traj', f_spline, 'max_traj', f_spline, 'traj', f_spline);
        kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,f_spline,tspan,'', ...
          ActionKinematicConstraint.NOT_IN_CONTACT, ...
          ActionKinematicConstraint.NOT_IN_CONTACT, ...
          ActionKinematicConstraint.NOT_IN_CONTACT);
        crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
      end
      
    elseif any(i == [2,4])
      fpos = fpos_start;
      % Initial stance constraints
      %tspan = [t_start/2,t_start+stride_duration/2];
      tspan = [0,t_start+stride_duration/2];
      kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,fpos(:,i), tspan,'',...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
        ActionKinematicConstraint.BREAK_CONTACT);
      crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
      if options.draw
        sfigure(7); hold on; grid on;
        plot(tspan,[i i],'s-');
        sfigure(8); hold on; grid on;
        plot(fpos(1,i),fpos(2,i),'bo');
      end
      if options.direction == 0
        fpos(:,i) = fpos(:,i) + forward_dir*options.step_length;
      else
        fpos(:,i) = fpos_all{2}(:,i);
      end

      % Intermediate stance constraints
      for j = 1:options.num_strides-1
        tspan = t_start + ((j-1)+1/2)*stride_duration + [swing_duration,stride_duration];
        kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,fpos(:,i), tspan,'', ...
          ActionKinematicConstraint.MAKE_CONTACT, ...
          ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
          ActionKinematicConstraint.BREAK_CONTACT);
        crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
        if options.draw
          sfigure(7); hold on; grid on;
          plot(tspan,[i i],'s-');
          sfigure(8); hold on; grid on;
          plot(fpos(1,i),fpos(2,i),'bo');
        end
        if options.direction == 0
          fpos(:,i) = fpos(:,i) + forward_dir*options.step_length;
        else
          fpos(:,i) = fpos_all{j+2}(:,i);
        end
      end

      % Final stance constraints
      j = options.num_strides;
      tspan = t_start + [((j-1)+1/2)*stride_duration + swing_duration, j*stride_duration];
      kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,fpos(:,i), tspan,'', ...
        ActionKinematicConstraint.MAKE_CONTACT, ...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT, ...
        ActionKinematicConstraint.STATIC_PLANAR_CONTACT);
      crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
      if options.draw
        sfigure(7); hold on; grid on;
        plot(tspan,[i i],'s-');
        sfigure(8); hold on; grid on;
        plot(fpos(1,i),fpos(2,i),'bo');
      end

      % Apex constraints
      fpos = fpos_start;
      j = 1;
      tspan = t_start + ((j-1)+1/2)*stride_duration + [0,swing_duration];
      x0_foot = fpos(:,i);
      if options.direction == 0
        fpos(:,i) = fpos(:,i) + forward_dir* options.step_length;
      else
        fpos(:,i) = fpos_all{2}(:,i);
      end
      xf_foot = fpos(:,i);
      f_spline = PPTrajectory(swing_foot_spline(x0_foot,xf_foot,options.step_height,tspan(2)-tspan(1),tspan(1)));
      link_constraints(end+1) = struct('link_ndx', foot_spec(i).body_ind, 'pt', body.contact_pts(:,foot_spec(i).contact_pt_ind), 'min_traj', f_spline, 'max_traj', f_spline, 'traj', f_spline);
      kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,f_spline, tspan,'', ...
        ActionKinematicConstraint.NOT_IN_CONTACT, ...
        ActionKinematicConstraint.NOT_IN_CONTACT, ...
        ActionKinematicConstraint.NOT_IN_CONTACT);
      crawl_sequence = addKinematicConstraint(crawl_sequence,kc);

      for j = 2:options.num_strides
        tspan = t_start + ((j-1)+1/2)*stride_duration + [0,swing_duration];
        x0_foot = fpos(:,i);
        if options.direction == 0
          fpos(:,i) = fpos(:,i) + forward_dir*options.step_length;
        else
          fpos(:,i) = fpos_all{j+1}(:,i);
        end
        xf_foot = fpos(:,i);
        f_spline = PPTrajectory(swing_foot_spline(x0_foot,xf_foot,options.step_height,tspan(2)-tspan(1),tspan(1)));
        link_constraints(end+1) = struct('link_ndx', foot_spec(i).body_ind, 'pt', body.contact_pts(:,foot_spec(i).contact_pt_ind), 'min_traj', f_spline, 'max_traj', f_spline, 'traj', f_spline);
        kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,f_spline, tspan,'', ...
          ActionKinematicConstraint.NOT_IN_CONTACT, ...
          ActionKinematicConstraint.NOT_IN_CONTACT, ...
          ActionKinematicConstraint.NOT_IN_CONTACT);
        crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
      end
    end
  end

  for i = 1:options.num_strides
    support_times((i-1)*4+1) = t_start + (i-1)*stride_duration;
    support_times((i-1)*4+2) = t_start + (i-1)*stride_duration + swing_duration;
    support_times((i-1)*4+3) = t_start + ((i-1)+0.5)*stride_duration;
    support_times((i-1)*4+4) = t_start + ((i-1)+0.5)*stride_duration + swing_duration;
    supports((i-1)*4+1) = SupportState(r,[foot_spec([2,4]).body_ind], ...
                                  {foot_spec([2,4]).contact_pt_ind},zeros(2,1));
    supports((i-1)*4+2) = SupportState(r,[foot_spec(1:4).body_ind], ...
                                  {foot_spec(1:4).contact_pt_ind},zeros(4,1));
    supports((i-1)*4+3) = SupportState(r,[foot_spec([1,3]).body_ind], ...
                                  {foot_spec([1,3]).contact_pt_ind},zeros(2,1));
    supports((i-1)*4+4) = SupportState(r,[foot_spec(1:4).body_ind], ...
                                  {foot_spec(1:4).contact_pt_ind},zeros(4,1));
  end
  
  for i = 1:numel(crawl_sequence.key_time_samples)
    support_vert{i} = getSupportPolygon(crawl_sequence,crawl_sequence.key_time_samples(i));
  end
  support_vert{end+1} = getSupportPolygon(crawl_sequence,t_start+options.num_strides*stride_duration);
  zmptraj = PPTrajectory(foh([t_start,t_start+options.num_strides*stride_duration], ...
  [com_start(1:2),mean(fpos_all{end}(1:2,:),2)]));
%   for i = 2:numel(crawl_sequence.key_time_samples)-1
  for i = 1:numel(support_times)
    %p1 = support_vert{i}(1:2,1);
    %p2 = support_vert{i}(1:2,2);
    %q1 = support_vert{i+1}(1:2,1);
    %q2 = support_vert{i+1}(1:2,2);
    %zmp_coeff = [(p1-p2), (q2-q1)]\(q2-p2);
%     zmp(:,i) = zmp_coeff(1)*p1 + (1-zmp_coeff(1))*p2;
%     assert(all(zmp(:,i) == zmp_coeff(2)*q1 + (1-zmp_coeff(2))*q2), ...
%       'Computed intersection points are inconsistent in zmp generation');
    zmp_mean(:,i) = mean(support_vert{i}(1:2,:),2);
    zmp(:,i) = eval(zmptraj,crawl_sequence.key_time_samples(i));
%     sfigure(8); hold on; grid on;
    sfigure(8); hold on; grid on; axis equal;
    plot(zmp(1,i),zmp(2,i),'gd',zmp_mean(1,i),zmp_mean(2,i),'rd');%,support_vert{i}(1,1:2),support_vert{i}(2,1:2),'-rs',support_vert{i+1}(1,1:2),support_vert{i+1}(2,1:2),'-ks');
  end
  %zmp(:,end+1) = mean(support_vert{end}(1:2,:),2);
  %zmptraj = PPTrajectory(foh(support_times,zmp));
  support_times = [0, support_times];
  supports = [SupportState(r,[foot_spec(1:4).body_ind], ...
                              {foot_spec(1:4).contact_pt_ind},zeros(4,1)),supports];

  zmptraj = setOutputFrame(zmptraj,desiredZMP);
  options.com0 = com_start(1:2);
  [~,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(options.com_height,zmptraj,options);
  %comtraj = [comtraj;ConstantTrajectory(com_start(3))];
  comtraj = [comtraj;ConstantTrajectory(NaN)];
  
  % COM constraint
  kc = ActionKinematicConstraint(r,0,[0;0;0],comtraj,comtraj.tspan,'crawling_COM_constraint');
  crawl_sequence = addKinematicConstraint(crawl_sequence,kc);

  comtraj_initial = PPTrajectory(foh([0, t_start/2, t_start],[com_initial, com_initial, com_start]));
  kc = ActionKinematicConstraint(r,0,[0;0;0],comtraj_initial,comtraj_initial.tspan,'transient_COM_constraint');
  crawl_sequence = addKinematicConstraint(crawl_sequence,kc);

  % COM constraint
  kc = ActionKinematicConstraint(r,pelvis_ind,[0;0;0],ConstantTrajectory([NaN;NaN;com_start(3)]),comtraj.tspan,'crawling_pelvis_constraint');
  crawl_sequence = addKinematicConstraint(crawl_sequence,kc);

  %Head constraint
  %kc = ActionKinematicConstraint(r,pelvis_ind,[0;0;0],struct('min',[-Inf;-Inf;pelvis_pos_initial(3)],'max',Inf(3,1)),[0, t_start],'initial pelvis constraint');
  %crawl_sequence = addKinematicConstraint(crawl_sequence,kc);

  % Initial end-effector trajectories
  for i = 1:4
    %f_spline = PPTrajectory(swing_foot_spline(fpos_initial(:,i),fpos_start(:,i),options.step_height,t_start/2,0));
    %kc = ActionKinematicConstraint(r,foot_spec(i).body_ind,body_pts,f_spline, [0,t_start/2],'', ...
      %ActionKinematicConstraint.NOT_IN_CONTACT, ...
      %ActionKinematicConstraint.NOT_IN_CONTACT, ...
      %ActionKinematicConstraint.NOT_IN_CONTACT);
    %crawl_sequence = addKinematicConstraint(crawl_sequence,kc);
  end
  
  t = 0:0.1:comtraj.tspan(2);
  nt = numel(t);
  q = zeros(nq,nt);
  q(:,1) = x0(1:nq);
  qtraj_initial = PPTrajectory(foh([0,t_start],[q(:,1),[x0(1:3);q_nom(4:end)]]));
  options.q_nom = q_nom;
  q0 = q_nom;
  for i = 2:nt
    if t(i) < t_start
      i_start = i+1;
    else
      ikargs = getIKArguments(crawl_sequence,t(i));
      q(:,i) = inverseKin(r,q0,ikargs{:},options);
      %q(:,i) = approximateIK(r,q0,ikargs{:},options);
      q0 = q(:,i);
    end
    if options.draw
      %v.draw(t(i),[q(:,i);0*q(:,i)]);
      %AtlasCommandPublisher(mex_ptr,'ATLAS_COMMAND',0,q(actuated,i));
      %pause(2);
    end
  end
  qtraj_initial = PPTrajectory(foh([0,t_start],[q(:,1),q(:,i_start+1)]));
  for i = 2:i_start-1
      ikargs = getIKArguments(crawl_sequence,t(i));
      options.q_nom = eval(qtraj_initial,t(i));
      q(:,i) = inverseKin(r,q(:,i-1),ikargs{:},options);
      %q(:,i) = approximateIK(r,q(:,i-1),ikargs{:},options);
      %q(:,i) = eval(qtraj_initial,t(i));
  end
  qdtraj = PPTrajectory(spline(t,q));
  %for step=1:2:options.num_steps
    %for swing_legs= [[1;3],[2;4]]
      %stance_legs = 1:4; stance_legs(swing_legs)=[];

      %% todo: add constraints to crawl sequence
      
      %% apex of swing leg
      %if (step==1 || step==options.num_steps)
        %zmp(1) = zmp(1)+ options.step_length/8;
        %fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/4;
      %else
        %zmp(1) = zmp(1) + options.step_length/4;
        %fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/2;
      %end
      %fpos(3,swing_legs) = options.step_height;
      %com = [zmp;options.com_height]; % note just set com=zmp here to get started 
      %q = crawlIK(q,com,fpos,swing_legs);  
      %display(q,com,fpos,swing_legs);
      
      %% end of step
      %if (step==1 || step==options.num_steps)
        %zmp(1) = zmp(1)+ options.step_length/8;
        %fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/4;
      %else
        %zmp(1) = zmp(1) + options.step_length/4;
        %fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/2;
      %end
      %fpos(3,swing_legs) = 0;
      %com = [zmp;options.com_height]; % note just set com=zmp here to get started 
      %q = crawlIK(q,com,fpos);  
      %display(q,com,fpos,swing_legs);
      %step=step+1;
    %end
  %end
end

return;

% 
%       startpos = fpos(:,leg);
%       if (step<=4 || options.num_steps-step<=4)
%         support_times(step+1) = support_times(step)+step_time/2;
%         if (0) %step<=4)
%           % move to comfortable fpos described by foot_right_foot
%           fpos(1:2,leg) = center0 + options.comfortable_footpos(:,leg) + [options.step_length/2;0];
%         else
%           fpos(1,leg) = fpos(1,leg)+options.step_length/2;
%         end
%       else
%         support_times(step+1) = support_times(step)+step_time;
%         fpos(1,leg) = fpos(1,leg)+options.step_length;
%       end
%       if ~options.ignore_terrain
%         fpos(3,leg) = getTerrainHeight(r,fpos(1:2,leg));
%       end
%       supports = horzcat(supports,SupportState(r,[foot_spec(suppind).body_ind],{foot_spec(suppind).contact_pt_ind}));
%       zmp_knots(:,step+1) = mean(fpos(1:2,suppind),2);
%       
%       if (options.draw && options.debug)
%         figure(1); clf; hold on; axis([-2,2,-2,2]); axis equal;
%         plot(fpos(1,:),fpos(2,:),'go');
%         plot(zmp_knots(1,step+1),zmp_knots(2,step+1),'rx');
%         pause(1);
%       end
%       support_pos{step+1} = fpos(:,suppind);
%       swing_leg(step) = leg;
%       swing_leg_trajs{step} = PPTrajectory(swing_foot_spline(startpos,fpos(:,leg),options.step_height,support_times(step+1)-support_times(step),support_times(step)));
%       step = step+1;
%     end
%   end
% end
% 
% support_times(end) = support_times(end-1)+step_time/2;
% supports = horzcat(supports,SupportState(r,[foot_spec.body_ind],{foot_spec.contact_pt_ind}));
% zmp_knots(:,end) = mean(fpos(1:2,:),2);
% support_pos{step+1} = fpos;
% 
% if (options.draw && options.debug)
%   figure(1); clf; hold on; axis([-2,2,-2,2]); axis equal; 
%   plot(fpos(1,:),fpos(2,:),'go');
%   plot(zmp_knots(1,end),zmp_knots(2,end),'rx','MarkerSize',4);
%   drawnow;
% end
% 
% zmptraj = setOutputFrame(PPTrajectory(foh(support_times,zmp_knots)),desiredZMP);
% com = getCOM(r,kinsol);
% options.com0 = com(1:2);
% [~,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(options.com_height,zmptraj,options);
% 
% if (options.draw && options.debug)
%   figure(1); hold on; axis([-2,2,-2,2]); axis equal; 
%   h=fnplt(zmptraj,[1 2]); set(h,'Color','r');
%   h=fnplt(comtraj,[1 2]); set(h,'Color','k');
%   plot(com(1),com(2),'kx');
%   legend('zmp','com');
% end




end

function support_vert = getSupportPolygon(action_sequence, t)
  support_vert = [];
  for j = 1:length(action_sequence.kincons)
    if t >= action_sequence.kincons{j}.tspan(1) && ...
       t <= action_sequence.kincons{j}.tspan(2) && ...
        any(any(cell2mat(action_sequence.kincons{j}.getContactState(t)') == ActionKinematicConstraint.STATIC_PLANAR_CONTACT,1) | ...
        any(cell2mat(action_sequence.kincons{j}.getContactState(t)') == ActionKinematicConstraint.STATIC_GRIP_CONTACT,1))
      support_vert = [support_vert, eval(action_sequence.kincons{j}.pos_min,t)];
    end
  end
end

function f_spline = swing_foot_spline(x0,xf,step_height,swing_duration,start_time)
% one of the swing foot splines from littledog 

if (nargin < 5) start_time = 0; end

X(:,1) = x0;
X(:,2) = (xf + x0)/2 + [ 0; 0; step_height ];
X(:,3) = [x0(1:2) + 1.1*(xf(1:2)-x0(1:2)); xf(3)+0.5*step_height];
X(:,4) = xf;

T(1) = 0;
T(2) = 0.45 * swing_duration;
T(3) = 0.75 * swing_duration;
T(4) = swing_duration;

T = T+start_time;

f_spline = csape(T,X,'periodic');

end
