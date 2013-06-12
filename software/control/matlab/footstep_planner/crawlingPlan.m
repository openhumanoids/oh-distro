function [support_times,supports,V,comtraj,zmptraj,qdtraj] = crawlingPlan(r,x0,body_spec,foot_spec,options)
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

typecheck(r,{'RigidBodyManipulator','TimeSteppingRigidBodyManipulator'});
sizecheck(x0,[getNumStates(r) 1]);
nq = getNumDOF(r);
q_nom = x0(1:nq);
kinsol = doKinematics(r,q_nom);

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
if ~isfield(options,'step_length') options.step_length = .3; end
if ~isfield(options,'step_speed') options.step_speed = .5; end  
if ~isfield(options,'step_height') options.step_height = .2; end
if ~isfield(options,'com_height') options.com_height = .35; end
if ~isfield(options,'comfortable_footpos') options.comfortable_footpos = [-.7 -.7 .6 .6; .3 -.3 -.3 .3]; end
if ~isfield(options,'ignore_terrain') options.ignore_terrain = true; end  % todo: make this default to false
if ~isfield(options,'direction') options.direction = 0; end
if ~isfield(options,'gait') options.gait = 0; end
if ~isfield(options,'draw') options.draw = true; end
if ~isfield(options,'debug') options.debug = false; end

% always take 4 steps at a time
options.num_steps = 4*ceil(options.num_steps/4);
step_time = abs(options.step_length/options.step_speed);
actuated = getActuatedJoints(r);

persistent mex_ptr;

if options.draw
  v = r.constructVisualizer();

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

q = q_nom;
for i=1:4
  fpos(:,i) = forwardKin(r,kinsol,foot_spec(i).body_ind,foot_spec(i).contact_pt);
end
%hip0 = forwardKin(r,kinsol,body_spec.body_ind,body_spec.pt);
com = [mean(fpos(1:2,:),2);options.com_height];
q = crawlIK(q,com,fpos);
display(q,com,fpos,[]); pause(5);

if (options.gait==0) % quasi-static
  order = [4 2 3 1];
  for step=1:4:options.num_steps
    for swing_leg=order
      stance_legs = [1:swing_leg-1,swing_leg+1:4];
      
      % todo: add constraints to crawl sequence

      % prepare for step (move COM to support triangle)
      a = mean(fpos(1:2,stance_legs),2);
      b = fpos(1:2,swing_leg);
      com = [.9*a+.1*b;options.com_height];
      q = crawlIK(q,com,fpos);
      display(q,com,fpos,swing_leg);
      
      % apex of swing leg
      if (step==1 || step==options.num_steps)
        fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/4;
      else
        fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/2;
      end
      fpos(3,swing_leg)=options.step_height;
      q = crawlIK(q,com,fpos,swing_leg);
      display(q,com,fpos,swing_leg);
      
      % end of step
      if (step==1 || step==options.num_steps)
        fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/4;
      else
        fpos(1,swing_leg) = fpos(1,swing_leg)+options.step_length/2;
      end
      fpos(3,swing_leg)=0;
      q = crawlIK(q,com,fpos);
      display(q,com,fpos,swing_leg);
      step=step+1;
    end
  end
elseif (options.gait ==2) % trot
  zmp = com(1:2);
  for step=1:2:options.num_steps
    for swing_legs= [[1;3],[2;4]]
      stance_legs = 1:4; stance_legs(swing_legs)=[];

      % todo: add constraints to crawl sequence
      
      % apex of swing leg
      if (step==1 || step==options.num_steps)
        zmp(1) = zmp(1)+ options.step_length/8;
        fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/4;
      else
        zmp(1) = zmp(1) + options.step_length/4;
        fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/2;
      end
      fpos(3,swing_legs) = options.step_height;
      com = [zmp;options.com_height]; % note just set com=zmp here to get started 
      q = crawlIK(q,com,fpos,swing_legs);  
      display(q,com,fpos,swing_legs);
      
      % end of step
      if (step==1 || step==options.num_steps)
        zmp(1) = zmp(1)+ options.step_length/8;
        fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/4;
      else
        zmp(1) = zmp(1) + options.step_length/4;
        fpos(1,swing_legs) = fpos(1,swing_legs) + options.step_length/2;
      end
      fpos(3,swing_legs) = 0;
      com = [zmp;options.com_height]; % note just set com=zmp here to get started 
      q = crawlIK(q,com,fpos);  
      display(q,com,fpos,swing_legs);
      step=step+1;
    end
  end
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
