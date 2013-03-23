function [q,t_sample,info] = inverseKinSequence(obj,q0,action_sequence,options)
% attempts to solve the optimization problem
% min_(q1,q2,...,qk) 0.5*sum(qdot(i))'Qv*qdot(i)+(q(i)-q_nom)'Q*(q(i)-q_nom)
% subject to
%    q(t) is a polynomial interpolated by (q1, q2,...,qk)
%    action_sequence.tspan
%    qdot(0) = 0
%    qdot(tf) = 0;
%    q(t_i) satisfies the action_sequence at time t
%    q(t_i) satisfies that the COM is in the support polygon, this is
%    OPTIONAL

% @param q0 the initial pos
% @action_sequence an ActionSequence object, which specifies kinematic
% constraint for the whole time interval
% options Qv penalize the joint velocity
% options q_nom the nominal postures
% options Q penalizes the difference from the nominal postures
% options nSample number of samples for the q(t)
% options supportPolygonFlag true if we want the COM lies inside the
% support polygon

nq = obj.getNumDOF;
if ~isfield(options,'use_mex') options.use_mex = exist('inverseKinmex')==3; end
if(~isfield(options,'Qv')) options.Qv = eye(nq); end
if(~isfield(options,'q_nom')) options.q_nom = zeros(nq,1); options.Q = zeros(nq);end
if(~isfield(options,'Q')) options.Q = eye(nq); end
if(~isfield(options,'nSample')) options.nSample = 2; end
if(~isfield(options,'supportPolygonFlag')) options.supportPolygonFlag = false; end
Fmin = 0;
Fmax = inf;

t_sample = linspace(action_sequence.tspan(1),action_sequence.tspan(end),options.nSample+1);
sizecheck(t_sample,[1,options.nSample+1]);
dt = diff(t_sample);
dt_ratio = dt(1:end-1)./(dt(2:end));
body_ind = cell(1,options.nSample);
body_pos = cell(1,options.nSample);
world_pos = cell(1,options.nSample);
do_rot = cell(1,options.nSample);

support_polygon_flags = cell(1,options.nSample);
contact_tol = 1e-4;

iGfun = ones(nq*options.nSample,1);
jGvar = (1:nq*options.nSample)';
nF = 1;
for i = 1:options.nSample
    ikargs = action_sequence.getIKArguments(t_sample(i+1));
    j = 1;
    n = 1;
    while j<length(ikargs)
        if(isa(ikargs{j},'RigidBody'))
            ikargs{j} = find(obj.body==ikargs{j},1);
        end
        body_ind{i}(n) = ikargs{j};
        if(body_ind{i}(n) == 0)
            body_pos{i}{n} = [0;0;0];
            world_pos{i}{n} = ikargs{j+1};
            support_polygon_flags{i}(n) = false;
            j = j+2;
        else
            body_pos{i}{n} = ikargs{j+1};
            if(ischar(body_pos{i}{n})||numel(body_pos{i}{n})==1)
                body_pos{i}{n} = getContactPoints(obj.body(body_ind{i}(n)),body_pos{i}{n});
            end
            world_pos{i}{n} = ikargs{j+2};
            j = j+3;
            [rows,mi] = size(body_pos{i}{n});
            if(rows~=3) error('bodypos must be 3xmi');end
        end
        if(isstruct(world_pos{i}{n}))
            if(~isfield(world_pos{i}{n},'min')||~isfield(world_pos{i}{n},'max'))
                error('if worldpos is a struct, it must have fields .min and .max')
            end
            maxpos = world_pos{i}{n}.max;
            minpos = world_pos{i}{n}.min;
        else
            maxpos = world_pos{i}{n};
            minpos = world_pos{i}{n};
        end
        [rows,cols] = size(minpos);
        if(rows~=3 && rows~=6) error('world pos must have 3 or 6 rows');end
        if(body_ind{i}(n) ==0 &&rows~=3) error('com pos must have only 3 rows');end
        if(cols~=mi) error('worldpos must have the same number of elements as bodypos');end
        sizecheck(maxpos,[rows,mi]);
        
        minpos(isnan(minpos)) = -inf;
        maxpos(isnan(maxpos)) = inf;
        do_rot{i}(n) = (rows == 6);
        iGfun = [iGfun;nF+repmat((1:rows*cols)',nq,1)];
        jGvar = [jGvar;nq*(i-1)+reshape(repmat(1:nq,rows*cols,1),[],1)];
        nF = nF+rows*cols;
        support_polygon_flags{i}(end+(1:mi)) = maxpos(3,:)<contact_tol;
        Fmin = [Fmin;minpos(:)];
        Fmax = [Fmax;maxpos(:)];
        n = n+1;
    end
end

% Suppose the joint angles are interpolated using cubic splines, then the
% velocity of the sample knots equals to velocity_mat *[q(0);q(1);...;q(k)]
% velocity_mat1_diag1 = reshape([2*dt(1)*ones(nq,1) repmat(dt(1:end-1).*(2+2*dt_ratio),nq,1) 2*dt(end)*ones(nq,1)],[],1);
% velocity_mat1_diag2 = reshape([dt(1)*ones(nq,1) repmat(dt(1:end-1).*dt_ratio,nq,1)],[],1);
% velocity_mat1_diag3 = reshape(repmat(dt,nq,1),[],1);
velocity_mat1_diag1 = reshape([ones(nq,1) repmat(dt(1:end-1).*(2+2*dt_ratio),nq,1) ones(nq,1)],[],1);
velocity_mat1_diag2 = reshape([zeros(nq,1) repmat(dt(1:end-1).*dt_ratio,nq,1)],[],1);
velocity_mat1_diag3 = [reshape(repmat(dt(1:end-1),nq,1),[],1);zeros(nq,1)];
velocity_mat1 = sparse((1:nq*(options.nSample+1))',(1:nq*(options.nSample+1))',velocity_mat1_diag1)...
    +sparse((1:nq*(options.nSample))',nq+(1:nq*options.nSample)',velocity_mat1_diag2,nq*(options.nSample+1),nq*(options.nSample+1))...
    +sparse(nq+(1:nq*options.nSample)',(1:nq*options.nSample)',velocity_mat1_diag3,nq*(options.nSample+1),nq*(options.nSample+1));

% velocity_mat2_diag1 = reshape([-3*ones(nq,1) bsxfun(@times,3*ones(1,options.nSample-1)-3*dt_ratio.^2,ones(nq,1)) 3*ones(nq,1)],[],1);
% velocity_mat2_diag2 = reshape([3*ones(nq,1) bsxfun(@times,3*dt_ratio.^2,ones(nq,1))],[],1);
% velocity_mat2_diag3 = -3*ones(nq*options.nSample,1);
velocity_mat2_diag1 = reshape([zeros(nq,1) bsxfun(@times,3*ones(1,options.nSample-1)-3*dt_ratio.^2,ones(nq,1)) zeros(nq,1)],[],1);
velocity_mat2_diag2 = reshape([zeros(nq,1) bsxfun(@times,3*dt_ratio.^2,ones(nq,1))],[],1);
velocity_mat2_diag3 = [-3*ones(nq*(options.nSample-1),1);zeros(nq,1)];
velocity_mat2 = sparse((1:nq*(options.nSample+1))',(1:nq*(options.nSample+1))',velocity_mat2_diag1)...
    +sparse((1:nq*options.nSample)',nq+(1:nq*options.nSample)',velocity_mat2_diag2,nq*(options.nSample+1),nq*(options.nSample+1))...
    +sparse(nq+(1:nq*options.nSample)',(1:nq*options.nSample)',velocity_mat2_diag3,nq*(1+options.nSample),nq*(1+options.nSample));
velocity_mat = velocity_mat1\velocity_mat2;
% Augment the constraint that the velocity at the two ends are zero,
[velocity_mat_row,velocity_mat_col,velocity_mat_val] = find(velocity_mat([1:nq,end-nq+1:end],nq+1:end));
iAfun = nF+velocity_mat_row;
jAvar = velocity_mat_col;
A = velocity_mat_val;
Fmax = [Fmax;-velocity_mat([1:nq,end-nq+1:end],1:nq)*q0]; 
Fmin = [Fmin;-velocity_mat([1:nq,end-nq+1:end],1:nq)*q0];
whigh = repmat(obj.joint_limit_max,options.nSample,1);
wlow = repmat(obj.joint_limit_min,options.nSample,1);
w0 = repmat(q0,options.nSample,1);
nF = length(Fmin);
nG = length(iGfun);

snset('Major optimality tolerance=1e-3');
global SNOPT_USERFUN;
SNOPT_USERFUN =@(w) ik(w,obj,nq,options.nSample+1,nF,nG,options.Qv,options.Q,body_ind,body_pos,do_rot,q0,options.q_nom,velocity_mat);

% keyboard;
tic
[q,F,info] = snopt(w0,wlow,whigh,Fmin,Fmax,'snoptUserfun',0,1,A,iAfun,jAvar,iGfun,jGvar);
toc

q = [q0 reshape(q,nq,options.nSample)];
keyboard;
end

function [f,G] = ik(w,obj,nq,nT,nF,nG,Qv,Q,body_ind,body_pos,do_rot,q0,q_nom,velocity_mat)
f = zeros(nF,1);
G = zeros(nG,1);
q = reshape(w,nq,nT-1);
qdot = reshape(velocity_mat*[q0;w],nq,nT);
q_diff = q-bsxfun(@times,ones(1,nT-1),q_nom);
f(1) = 0.5*sum(sum((Qv*qdot(:,2:end-1)).*qdot(:,2:end-1)))+0.5*sum(sum((Q*q_diff).*q_diff));
G(1:nq*(nT-1)) = reshape(reshape((qdot(:,2:end-1)'*Qv)',1,[])*velocity_mat(nq+1:end-nq,nq+1:end),[],1)...
    +reshape(Q*q_diff,[],1);
if(nF<2) return; end
nf = 1;
ng = nq*(nT-1);
for i = 1:nT-1
    kinsol = doKinematics(obj,q(:,i));
    for j = 1:length(body_ind{i})
        if(body_ind{i}(j) == 0)
            [x,J] = getCOM(obj,kinsol);
        else
           [x,J] = forwardKin(obj,kinsol,body_ind{i}(j),body_pos{i}{j},do_rot{i}(j)); 
        end
        n = numel(x);
        f(nf+(1:n)) = x(:);
        G(ng+(1:n*nq)) = J(:);
        nf = nf+n;
        ng = ng+n*nq;
    end
end
end