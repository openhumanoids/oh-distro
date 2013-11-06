classdef TwoBodyRelativePositionConstraint < PositionConstraint
  properties(SetAccess = protected)
    body1
    body1_name
    body1_pt
    body2_pt
    body2
    body2_name
  end
  
  methods(Access = protected)
    function [pos,J] = evalPositions(obj,kinsol)
      [x1,J1] = forwardKin(obj.robot, kinsol, obj.body1, obj.body1_pt);
      [x2,J2] = forwardKin(obj.robot, kinsol, obj.body2, obj.body2_pt);
      pos = x2 - x1;
      J = J2 - J1;
    end
  end
  
  methods
    function obj = TwoBodyRelativePositionConstraint(robot,body1,body2,pts,lb,ub,tspan)
      if(nargin == 6)
        tspan = [-inf,inf];
      end
      
      sizecheck(pts,[3,2]);
      sizecheck(body1,[1,1]);
      sizecheck(body2,[1,1]);
      if(~isnumeric(body1) || ~isnumeric(body2))
        error('Drake:TwoBodyRelativePositionConstraint: body must be an integer');
      end

      obj = obj@PositionConstraint(robot, zeros(3,1), lb, ub,tspan);
      obj.body1 = floor(body1);
      obj.body2 = floor(body2);
      obj.body1_pt = pts(:,1);
      obj.body2_pt = pts(:,2);
      obj.body1_name = obj.robot.getBody(obj.body1).linkname;
      obj.body2_name = obj.robot.getBody(obj.body2).linkname;
    end

    

    function name_str = name(obj,t)
      if(isempty(t) || (t>=obj.tspan(1)&&t<=obj.tspan(end)))
        name_str = repmat({sprintf('Relative position constraint for %s and %s at time %10.4f',obj.body1_name,obj.body2_name,t)},obj.getNumConstraint(t),1);
      else
        name_str = [];
      end
    end

    function ptr = constructPtr(varargin)
      ptr = [];
    end

    function drawConstraint(obj,q,lcmgl)
%       kinsol = doKinematics(obj.robot,q,false,false);
%       pts_w = forwardKin(obj.robot,kinsol,1,obj.pts);
%       wTbp = kinsol.T{obj.bodyB.idx}*invHT(obj.bodyB.bpTb);
%       wPbp = wTbp(1:3,4);
%       bot_lcmgl_draw_axes(lcmgl);
%       for pt = pts_w
%         bot_lcmgl_color3f(lcmgl,0.25,0.25,0.25);
%         bot_lcmgl_sphere(lcmgl, pt, 0.02, 36, 36);
%       end
%       a = rotmat2axis(wTbp(1:3,1:3));
%       bot_lcmgl_translated(lcmgl,wPbp(1),wPbp(2),wPbp(3));
%       bot_lcmgl_rotated(lcmgl,a(4)*180/pi,a(1),a(2),a(3));
%       bot_lcmgl_draw_axes(lcmgl);
%       bot_lcmgl_color4f(lcmgl,0,1,0,0.5);
%       bot_lcmgl_box(lcmgl,(obj.lb+obj.ub)/2,obj.ub-obj.lb);
%       bot_lcmgl_switch_buffer(lcmgl);
    end

    function obj = updateRobot(obj,robot)
      obj.robot = robot;
%       updatePtrPoint2PointDistanceConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end
  end
end
