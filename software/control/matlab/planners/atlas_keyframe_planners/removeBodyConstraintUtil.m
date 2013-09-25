function new_constraint_cell = removeBodyConstraintUtil(tspan,old_constraint_cell)
  % The utility function to delete the constraint in
  % old_constraint_cell if it is active in tspan
  new_constraint_cell = {};
  for i = 1:length(old_constraint_cell)
    if(~(old_constraint_cell{i}.isTimeValid(tspan(1))||old_constraint_cell{i}.isTimeValid(tspan(end))))
      new_constraint_cell = [new_constraint_cell, old_constraint_cell(i)]; 
    end
  end
end