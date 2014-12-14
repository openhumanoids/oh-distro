function q = quat_mult(qa,qb)
%
% Q = QUAT_MULT(QA,QB) multiplies quaternion QA by quaternion QB.
%   Q = QUAT_MULT(QA,QB) returns the quaternion result of the mutiplication
%   of QA and QB. That is, Q = QA*QB. Also, quat_to_rot(Q) = 
%   quat_to_rot(quat_mult(QA,QB)). Note that quaternion multiplication is
%   not communicative. Per libbot convention, Q, QA, and QB are [w x y z].
%
%   Implementation derived from libbot's bot_quat_mult()
%

error(nargchk(2, 2, nargin));
if ndims(qa) > 2 || (isvector(qa) && length(qa) ~= 4) || (~isvector(qa) && size(qa,2) ~= 4)
    error('Invalid argument, QA must be a 4 element vector or Nx4 matrix.');
end
if ndims(qb) > 2 || (isvector(qb) && length(qb) ~= 4) || (~isvector(qb) && size(qb,2) ~= 4)
    error('Invalid argument, QB must be a 4 element vector or Nx4 matrix.');
end
if ~isvector(qa) && ~isvector(qb) && ~all(size(qa)==size(qb))
    error('Invalid argument, QA and QB must be the same size if both are matrices.');
end

t = [isvector(qa) && size(qa,2) == 1; isvector(qb) && size(qb,2) == 1];
if t(1)
    qa = qa';
end
if t(2)
    qb = qb';
end
do_transpose = all(t);

N = max(size(qa,1), size(qb,1));
if N > 1
    if isvector(qa)
        qa = repmat(qa, N, 1);
    end
    if isvector(qb)
        qb = repmat(qb, N, 1);
    end
end

q = zeros(N, 4);
q(:,1) = qa(:,1).*qb(:,1) - qa(:,2).*qb(:,2) - qa(:,3).*qb(:,3) - qa(:,4).*qb(:,4);
q(:,2) = qa(:,1).*qb(:,2) + qa(:,2).*qb(:,1) + qa(:,3).*qb(:,4) - qa(:,4).*qb(:,3);
q(:,3) = qa(:,1).*qb(:,3) - qa(:,2).*qb(:,4) + qa(:,3).*qb(:,1) + qa(:,4).*qb(:,2);
q(:,4) = qa(:,1).*qb(:,4) + qa(:,2).*qb(:,3) - qa(:,3).*qb(:,2) + qa(:,4).*qb(:,1);

if do_transpose
    q = q';
end

end