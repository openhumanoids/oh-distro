classdef RelativeFixedQuatConstraint < MultipleTimeKinematicConstraint
  % Constrain a certain body to be in a fixed orientation for a time
  % interval
  % @param robot           -- A RigidBodyManipulator or a
  %                           TimeSteppingRigidBodyManipulator
  % @param body            -- An int scalar, the body index
  % @param tspan           -- Optional input, a 1x2 double array. The time
  %                           span of this constraint being active. Default
  %                           is [-inf inf];
  properties(SetAccess = protected)
    body1
    body1_name
    body2
    body2_name
  end
  
  methods
    function obj = RelativeFixedQuatConstraint(robot,body1,body2,tspan)
      if(nargin == 3)
        tspan = [-inf inf];
      end
%       ptr = constructPtrWorldFixedOrientConstraintmex(robot.getMexModelPtr,body,tspan);
      obj = obj@MultipleTimeKinematicConstraint(robot,tspan);
      sizecheck(body1,[1,1]);
      sizecheck(body2,[1,1]);
      if(~isnumeric(body1) || ~isnumeric(body2))
        error('Drake:RelativeFixedQuatConstraint: body must be an integer');
      end
      obj.body1 = floor(body1);
      obj.body2 = floor(body2);
      obj.body1_name = obj.robot.getBody(obj.body1).linkname;
      obj.body2_name = obj.robot.getBody(obj.body2).linkname;
      obj.mex_ptr = [];
    end
    
    function num = getNumConstraint(obj,t)
      valid_t = t(obj.isTimeValid(t));
      if(length(valid_t)>=2)
        num = 1;
      else
        num = 0;
      end
    end
    
    function [c,dc_valid] = eval_valid(obj,valid_t,valid_q)
      num_valid_t = size(valid_t,2);
      nq = obj.robot.getNumDOF();
      sizecheck(valid_q,[nq,num_valid_t]);
      quat_rel = zeros(4,num_valid_t);
      if(nargout == 2)
        J = zeros(4*num_valid_t,nq);
      end
      for i = 1:num_valid_t
        kinsol = doKinematics(obj.robot,valid_q(:,i),false,false);
        if(nargout == 1)
          pos_tmp1 = forwardKin(obj.robot,kinsol,obj.body1,[0;0;0],2);
          pos_tmp2 = forwardKin(obj.robot,kinsol,obj.body2,[0;0;0],2);
          quat_rel(:,i) = quatDiff(pos_tmp1(4:7), pos_tmp2(4:7))
        elseif(nargout == 2)
          [pos_tmp1,J_tmp1] = forwardKin(obj.robot,kinsol,obj.body1,[0;0;0],2);
          [pos_tmp2,J_tmp2] = forwardKin(obj.robot,kinsol,obj.body2,[0;0;0],2);
          [quat_rel(:,i), dquat_rel] = quatDiff(pos_tmp1(4:7), pos_tmp2(4:7));
          J((i-1)*4+(1:4),:) = dquat_rel(:,1:4)*J_tmp1(4:7,:) + dquat_rel(:,5:8)*J_tmp2(4:7,:);
        end
      end
      quat2 = [quat_rel(:,2:end) quat_rel(:,1)];
      c1 = sum(quat_rel.*quat2,1);
      c = sum(c1.^2);
      if(nargout == 2)
        % [dcdquat1' dcdquat2' ...dcdquat_n_breaks'];
        dcdquat = (bsxfun(@times,ones(4,1),2*c1).*quat2+bsxfun(@times,ones(4,1),2*[c1(end) c1(1:end-1)]).*[quat_rel(:,end) quat_rel(:,1:end-1)]);
        dc_valid = sum(reshape(permute(reshape((bsxfun(@times,ones(1,nq),reshape(dcdquat,[],1)).*J)',nq,4,num_valid_t),[2,1,3]),4,nq*num_valid_t),1);
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      valid_t = t(obj.isTimeValid(t));
      if(length(valid_t)>=2)
        num_valid_t = length(valid_t);
        lb = num_valid_t;
        ub = num_valid_t;
      else
        lb = [];
        ub = [];
      end
    end
    
    function name_str = name(obj,t)
      valid_t = t(obj.isTimeValid(t));
      if(length(valid_t)>=2)
        name_str = {sprintf('Relative fixed orientation constraint for %s and %s',obj.body1_name,obj.body2_name)};
      else
        name_str = {};
      end
    end
    
    function obj = updateRobot(obj,robot)
      obj.robot = robot;
%       updatePtrWorldFixedOrientConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end
    
    function ptr = constructPtr(varargin)
%       ptr = constructPtrWorldFixedOrientConstraintmex(varargin{:});
      ptr = [];
    end
  end
end