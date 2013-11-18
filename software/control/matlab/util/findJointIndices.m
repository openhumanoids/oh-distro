function [ indices ] = findJointIndices( rbm, str )
  %findJointIndices Returns indices in the state vector for joints whose
  %name contains a specified string.
  %   @param str String to be searched for
  %   @retvall indices Array of indices into state vector
  nq = rbm.getNumDOF();
  indices = find(~cellfun('isempty',strfind(rbm.getStateFrame().coordinates(1:nq),str)));
end

