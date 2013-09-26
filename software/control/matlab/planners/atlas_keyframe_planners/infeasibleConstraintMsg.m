function infeasible_constraint_msg = infeasibleConstraintMsg(infeasible_constraint)
% The utility function to print the infeasible constraints to string
% @param infeasible_constraint        A cell array, each entry is the
%                                     string of infeasible constraint name
% @retval infeasible_constraint_msg   A string of all the infeasible
%                                     constraint
if(isempty(infeasible_constraint))
  infeasible_constraint_msg = '';
else
  if(iscell(infeasible_constraint))
    infeasible_constraint_msg = sprintf('The infeasible constraints are:\n');
    for i = 1:length(infeasible_constraint)
      infeasible_constraint_msg = sprintf('%s %s\n',infeasible_constraint_msg,infeasible_constraint{i});
    end
  else
    error('infeasible_constraint has to be a cell, each entry being the constraint name');
  end
end
end