function [q,info,infeasible_constraint] = inverseKinRepeatSearch(obj,total_ik_attempt,q_seed,q_nom,varargin)
% The same input and output as inverseKin. But search the solution repetitvely with some
% random guess
% @param total_ik_attempt   The maximum number of IK trials
ik_attempt_count = 0;
findFinalPostureFlag = false;
while(~findFinalPostureFlag && ik_attempt_count< total_ik_attempt)
  [q,info,infeasible_constraint] = inverseKin(obj,q_seed,q_nom,varargin{:});
  ik_attempt_count = ik_attempt_count+1;
  q_seed = q+5e-1*randn(size(q));
  findFinalPostureFlag = info<=10;
end
if(findFinalPostureFlag)
  display(sprintf('The IK succeeds after %d trials',ik_attempt_count));
else
  display(sprintf('The IK fails after %d trials',ik_attempt_count));
end
end