function [support_times,supports,V,comtraj,zmptraj,qtraj] = crawlingPlan(r,x0,body_spec,foot_spec,options)
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
% @option com_height in m
% @option front_right_foot  xy vector from body to nominal front right foot 
% @option ignore_terrain
% @options direction - 0 for forward, <0 for left, >0 for right 
% @options gait - 0 for walk, 1 for trot

typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
sizecheck(x0,[getNumStates(r) 1]);
nq = getNumDOF(r);
q0 = x0(1:nq);
kinsol = doKinematics(r,q0);

fieldcheck(body_spec,'body_ind');
fieldcheck(body_spec,'pt');
sizecheck(body_spec.pt,[3 1]);

sizecheck(foot_spec,4);
fieldcheck(foot_spec,'body_ind');
fieldcheck(foot_spec,'contact_pt_ind');
for i=1:4, foot_spec(i).contact_pt_ind = foot_spec(i).contact_pt_ind(1); end

if nargin<3 || isempty(options), options=struct(); end
if ~isfield(options,'num_steps') options.num_steps = 20; end
if ~isfield(options,'step_length') options.step_length = .3; end
if ~isfield(options,'step_speed') options.step_speed = .5; end  
if ~isfield(options,'step_height') options.step_height = .2; end
if ~isfield(options,'com_height') options.com_height = .25; end
if ~isfield(options,'comfortable_footpos') options.comfortable_footpos = [-.7 -.7 .6 .6; .3 -.3 -.3 .3]; end
if ~isfield(options,'ignore_terrain') options.ignore_terrain = true; end  % todo: make this default to false
if ~isfield(options,'direction') options.direction = 0; end
if ~isfield(options,'q_nom') options.q_nom = q0; end
if ~isfield(options,'gait') options.gait = 0; end
if ~isfield(options,'draw') options.draw = true; end
if ~isfield(options,'debug') options.debug = false; end

% always take 4 steps at a time
options.num_steps = 4*ceil(options.num_steps/4);
hip0 = forwardKin(r,kinsol,body_spec.body_ind,body_spec.pt);
step_time = abs(options.step_length/options.step_speed);

if options.draw
  v = r.constructVisualizer();
  v.draw(0,x0);
end

support_pos = cell(1,options.num_steps+2);
swing_leg = zeros(1,options.num_steps);
swing_leg_trajs = cell(1,options.num_steps);

support_times = zeros(1,options.num_steps+2);
supports(1) = SupportState(r,[foot_spec.body_ind],{foot_spec.contact_pt_ind});
fpos = contactPositions(supports(1),r,kinsol);
zmp_knots = repmat(mean(fpos(1:2,:),2),1,options.num_steps+2);
support_pos{1} = fpos;

center0 = mean(fpos(1:2,:),2);

if (options.draw && options.debug)
  figure(1); clf; hold on; axis([-2,2,-2,2]); axis equal; 
  plot(fpos(1,:),fpos(2,:),'go');
  plot(zmp_knots(1,1),zmp_knots(2,1),'rx');
  pause(1);
end


if (options.gait==0) % 
  order = [1 4 2 3];
  for step=1:4:options.num_steps
    for leg=order
      startpos = fpos(:,leg);
      if (step<=4 || options.num_steps-step<=4)
        support_times(step+1) = support_times(step)+step_time/2;
        if (0) %step<=4)
          % move to comfortable fpos described by foot_right_foot
          fpos(1:2,leg) = center0 + options.comfortable_footpos(:,leg) + [options.step_length/2;0];
        else
          fpos(1,leg) = fpos(1,leg)+options.step_length/2;
        end
      else
        support_times(step+1) = support_times(step)+step_time;
        fpos(1,leg) = fpos(1,leg)+options.step_length;
      end
      if ~options.ignore_terrain
        fpos(3,leg) = getTerrainHeight(r,fpos(1:2,leg));
      end
      suppind = [1:leg-1,leg+1:4];
      supports = horzcat(supports,SupportState(r,[foot_spec(suppind).body_ind],{foot_spec(suppind).contact_pt_ind}));
      zmp_knots(:,step+1) = mean(fpos(1:2,suppind),2);
      
      if (options.draw && options.debug)
        figure(1); clf; hold on; axis([-2,2,-2,2]); axis equal;
        plot(fpos(1,:),fpos(2,:),'go');
        plot(zmp_knots(1,step+1),zmp_knots(2,step+1),'rx');
        pause(1);
      end
      support_pos{step+1} = fpos(:,suppind);
      swing_leg(step) = leg;
      swing_leg_trajs{step} = PPTrajectory(swing_foot_spline(startpos,fpos(:,leg),options.step_height,support_times(step+1)-support_times(step),support_times(step)));
      step = step+1;
    end
  end
end

support_times(end) = support_times(end-1)+step_time/2;
supports = horzcat(supports,SupportState(r,[foot_spec.body_ind],{foot_spec.contact_pt_ind}));
zmp_knots(:,end) = mean(fpos(1:2,:),2);
support_pos{step+1} = fpos;

if (options.draw && options.debug)
  figure(1); clf; hold on; axis([-2,2,-2,2]); axis equal; 
  plot(fpos(1,:),fpos(2,:),'go');
  plot(zmp_knots(1,end),zmp_knots(2,end),'rx','MarkerSize',4);
  drawnow;
end

zmptraj = setOutputFrame(PPTrajectory(foh(support_times,zmp_knots)),desiredZMP);
com = getCOM(r,kinsol);
options.com0 = com(1:2);
[~,V,comtraj] = LinearInvertedPendulum.ZMPtrackerClosedForm(options.com_height,zmptraj,options);

if (options.draw && options.debug)
  figure(1); hold on; axis([-2,2,-2,2]); axis equal; 
  h=fnplt(zmptraj,[1 2]); set(h,'Color','r');
  h=fnplt(comtraj,[1 2]); set(h,'Color','k');
  plot(com(1),com(2),'kx');
  legend('zmp','com');
end


%% create desired joint trajectory
cost = Point(getStateFrame(r),1);
cost.base_x = 0;
cost.base_y = 0;
cost.base_z = 0;
cost.base_roll = 10;
cost.base_pitch = 0;
cost.base_yaw = 0;
%cost.back_lbz = 10;
%cost.back_mby = 100;
%cost.back_ubx = 100;
cost = double(cost);
%options = struct();
options.Q = diag(cost(1:nq));

q = q0;
for t=0:.025:support_times(end)-eps
  args = {0, [eval(comtraj,t);options.com_height]}; % com
  supp_tind = find(t>=support_times,1,'last');
  if (supp_tind>1 && supp_tind<length(support_times)-1)
    swing_foot_spec = foot_spec(swing_leg(supp_tind));
    swing_foot_pts = getContactPoints(getLink(r,swing_foot_spec.body_ind));
    swing_foot_pts = swing_foot_pts(:,swing_foot_spec.contact_pt_ind);
    args = horzcat(args,{swing_foot_spec.body_ind, swing_foot_pts(:,swing_foot_spec.contact_pt_ind), eval(swing_leg_trajs{supp_tind},t)}); % swing foot
  end
  for i=1:length(supports(supp_tind).bodies)
    pts = getContactPoints(getLink(r,supports(supp_tind).bodies(i)));
    pts = pts(:,supports(supp_tind).contact_pts{i});
    args = horzcat(args,{supports(supp_tind).bodies(i),pts,support_pos{supp_tind}(:,i)});
  end
  % add gaze constraints
%  args = horzcat(args,find_link(');
  
  q = inverseKin(r,q,args{:});
  
  if (options.draw)
    v.draw(t,[q;0*q]); drawnow;
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
