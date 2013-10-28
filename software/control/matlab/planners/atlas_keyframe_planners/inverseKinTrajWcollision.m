function [xtraj,info,infeasible_constraint,joint_limit_flag] = inverseKinTrajWcollision(obj,collision_status,t_verify, ...
  t_breaks,q_seed_traj,q_nom_traj,varargin)
% The interface is the same as inverseKinTraj
% @param collision_status             - 0, no validation, no optimizatoin with collision
%                                     - 1, validation only, no optimization
%                                     - 2, optimize without collision constraint first, then validate. 
%                                          if collision happens, optimization with collision constraint
% @param t_verify                     - The additional time samples to verify if the constraints
%                                       (collision avoidance) are satisfied
% @param joint_limit_flag             - The flag of whether the joint are at the joint limit in the
%                                       xtraj
t_verify = sort([t_verify t_breaks]);
nq = obj.getNumDOF();
[joint_lb,joint_ub] = obj.getJointLimits();
joint_max = bsxfun(@times,joint_ub,ones(1,length(t_breaks)));
joint_min = bsxfun(@times,joint_lb,ones(1,length(t_breaks)));
if(isa(varargin{end},'IKoptions'))
  ikoptions = varargin{end};
  varargin = varargin(1:end-1); 
else
  ikoptions = IKoptions(obj);
end
collision_constraint_cell = {};
other_constraint_cell = {};

for i = 1:length(varargin)
  if(isa(varargin{i},'AllBodiesClosestDistanceConstraint'))
    collision_constraint_cell = [collision_constraint_cell varargin(i)];
  elseif(isa(varargin{i},'DrakeMexPointer'))
    if strcmp(varargin{i}.name,'AllBodiesClosestDistanceConstraint')
      error('Please construct MATLAB AllBodiesClosestDistanceConstraint object');
%     elseif strcmp(varargin{i}.name,'PostureConstraint')
%       [joint_lb,joint_ub] = testPostureConstraintmex(varargin{i},t_breaks);
%       joint_min = reshape(max([joint_min(:) joint_lb(:)],[],2),nq,[]);
%       joint_max = reshape(min([joint_max(:) joint_ub(:)],[],2),nq,[]);
    end
  else
    other_constraint_cell = [other_constraint_cell,varargin(i)];
%     if(isa(varargin{i},'PostureConstraint'))
%       [joint_lb,joint_ub] = varargin{i}.bounds(t_breaks);
%       joint_min = reshape(max([joint_min(:) joint_lb(:)],[],2),nq,[]);
%       joint_max = reshape(min([joint_max(:) joint_ub(:)],[],2),nq,[]);
%     end
  end
end
[xtraj,info,infeasible_constraint] = inverseKinTraj(obj,t_breaks,q_seed_traj,q_nom_traj,other_constraint_cell{:},ikoptions);

x_sol = xtraj.eval(t_breaks);
q_sol = x_sol(1:nq,:);
joint_limit_flag = abs(q_sol-joint_max)<1e-5 | abs(q_sol-joint_min)<1e-5;
coords = obj.getStateFrame.coordinates;
if(ikoptions.fixInitialState)
  t_start_idx = 2;
else
  t_start_idx = 1;
end
for i = t_start_idx:length(t_breaks)
  for j = 1:nq
    if(joint_limit_flag(j,i))
      display(sprintf('%s is at joint limit at time %4.2f',coords{j},t_breaks(i)));
    end
  end
end

collisionAvoidFlag = false(length(t_verify),length(collision_constraint_cell));
if(collision_status == 1 || collision_status == 2)
  for i = 1:length(t_verify)
    xi = xtraj.eval(t_verify(i));
    qi = xi(1:nq);
    for j = 1:length(collision_constraint_cell)
      if(collision_constraint_cell{j}.isTimeValid(t_verify(i)))
        [collisionAvoidFlag(i,j), dist,ptsA,ptsB,idxA,idxB] = collision_constraint_cell{j}.checkConstraint(qi);
        if(~collisionAvoidFlag(i,j))
          for k = 1:length(dist)
            send_status(4,0,0,sprintf('t=%4.2f,Dist from %s to %s is %f\n',...
              t_verify(i),...
              sendNameString(collision_constraint_cell{j},idxA),...
              sendNameString(collision_constraint_cell{j},idxB),...
              dist(k)));
          end
        end
      end
    end
  end
end
if(collision_status == 2 && ~all(collisionAvoidFlag))
  [xtraj,info,infeasible_constraint] = inverseKinTraj(obj,t_breaks,q_seed_traj,q_nom_traj,collision_constraint_cell{:},other_constraint_cell{:},ikoptions);
end
end

function name_str = sendNameString(collision_constraint,body_ind)
robotnum = collision_constraint.robot.getBody(body_ind).robotnum;
if(robotnum == 1) % atlas
  name_str = collision_constraint.robot.getBody(body_ind).linkname;
elseif(robotnum == 0) % world
  name_str = 'world';
else % affordance
  name_str = collision_constraint.robot.name{robotnum};
end
end