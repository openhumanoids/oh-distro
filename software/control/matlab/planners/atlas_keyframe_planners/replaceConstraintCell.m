function constraint_cell = replaceConstraintCell(old_constraint_cell,new_constraint_cell)
  % The utility function to replace the old constraint cell by a
  % new constraint cell for a given body. It should replace the old Position and
  % Orientation constraint by the new one (if exists). Also it
  % would replace the GazeConstraint. The constraint would be
  % appended to the old one
  constraint_cell = new_constraint_cell;
  for i_old = 1:length(old_constraint_cell)
    old_constraint = old_constraint_cell{i_old};
    old_tspan = old_constraint.getTspan();
    replaceFlag = false; % True if we need to delete this old constraint
    for i_new = 1:length(new_constraint_cell)
      new_constraint = new_constraint_cell{i_new};
      if(new_constraint.isTimeValid(old_tspan(1))||new_constraint.isTimeValid(old_tspan(2)))
        % overlap time span. possible to replace
        if(isa(old_constraint,'PositionConstraint') && isa(new_constraint,'PositionConstraint'))
          replaceFlag = true;
        elseif((isa(old_constraint,'QuatConstraint')||isa(old_constraint,'EulerConstraint'))...
            &&(isa(new_constraint,'QuatConstraint')||isa(new_constraint,'EulerConstraint')))
          replaceFlag = true;
        elseif(isa(old_constraint,'GazeConstraint') && isa(new_constraint,'GazeConstraint'))
          replaceFlag = true;
        end
      end
    end
    if(~replaceFlag)
      constraint_cell = [{old_constraint} constraint_cell];
    end
  end
end