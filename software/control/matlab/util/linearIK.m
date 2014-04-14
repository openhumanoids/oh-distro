function q = linearIK(r,q0,varargin)

% solves IK by linearizing kinematics and solving an equality constrained
% QP analytically (ignoring joint limits!) 

if isstruct(varargin{end}) 
  options = varargin{end};
  varargin=varargin(1:end-1);
else
  options = struct();
end

if isfield(options,'q_nom') q_nom = options.q_nom; else q_nom = q0; end
if isfield(options,'Q') Q = options.Q; else Q = eye(obj.num_q); end
if ~isfield(options,'use_mex') options.use_mex = true; end

i=1;
while i<=length(varargin)
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  body_ind=varargin{i}; 
  if (body_ind==0)
    i = i+2;
  else
    i = i+3;
  end
end

if options.use_mex
  q = linearIKmex(getMexModelPtr(r),q0,q_nom,Q,varargin{:});
  return;
end

kinsol = doKinematics(r,q0);

A={};b={};Ai={};bi={};
i=1;neq=1;nin=1;
while i<=length(varargin)
  % support input as bodies instead of body inds
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(r.body==varargin{i},1); end
  body_ind=varargin{i}; 
  
  if (body_ind==0)
    body_pos = [0;0;0];
    world_pos = varargin{i+1};
    i=i+2;
  else
    body_pos = varargin{i+1};
    world_pos = varargin{i+2};
    if ischar(body_pos) || numel(body_pos)==1 % then it's the name of a collision group
      b = r.body(body_ind);
      body_pos = mean(getContactPoints(b,body_pos),2);
    end
    i=i+3;
    [rows,~] = size(body_pos);
    if (rows ~=3) error('bodypos must be 3xmi'); end
  end
  
  if isstruct(world_pos)
    if ~isfield(world_pos,'min') || ~isfield(world_pos,'max')
      error('if world_pos is a struct, it must have fields .min and .max');
    end
    minpos=[world_pos.min];  maxpos=[world_pos.max];
    
    rows = size(minpos,1);
    maxrows = size(maxpos,1);
    if (rows ~= 3 && rows ~= 6) error('world_pos.min must have 3 or 6 rows'); end
    if (maxrows ~= rows) error('world_pos.max must have the same number of rows as world_pos.min'); end
  else
    minpos=[]; maxpos=[];
    [rows,~] = size(world_pos);
    if (rows ~= 3 && rows ~= 6 && rows ~= 7) error('worldpos must have 3, 6 or 7 rows'); end
  end
  if (body_ind==0 && rows ~= 3) error('com pos must have only 3 rows (there is no orientation)'); end
  
  if (body_ind==0)
    [x,J] = getCOM(r,kinsol);
  else
    [x,J] = forwardKin(r,kinsol,body_ind,body_pos,(rows==6));
    if rows==6 && isempty(minpos) && isempty(maxpos) 
      % make sure desired/current angles are unwrapped
      delta = angleDiff(x(4:6),world_pos(4:6));
      world_pos(4:6) = x(4:6)+delta;
    end
  end
    
  if isempty(minpos) && isempty(maxpos) 
    % add equality constraint
    idx = ~isnan(world_pos);
    A{neq} = J(idx,:);
    b{neq} = world_pos(idx) - x(idx) + J(idx,:)*q0;
    neq=neq+1;
  end
    
end

Aeq = sparse(vertcat(A{:}));
beq = vertcat(b{:});

f = -q_nom'*Q;

A = [Q Aeq'; Aeq zeros(size(Aeq,1))];
b = [-f'; beq];
y = A\b;
q = y(1:getNumDOF(r));

end