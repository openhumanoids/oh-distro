classdef WalkingPlan
  properties
    xtraj
    ts
    joint_names
  end

  methods
    function obj = WalkingPlan(ts, xtraj, joint_names)
      obj.ts = ts;
      obj.xtraj = xtraj;
      obj.joint_names = joint_names;
    end

    function msg = toLCM(obj)
      plan_pub = drc.control.RobotPlanPublisher(obj.joint_names,true,'CANDIDATE_ROBOT_PLAN');
      plan_pub.allocate(length(obj.ts));
      msg = plan_pub.encode(obj.ts, obj.xtraj);
    end
  end
end

