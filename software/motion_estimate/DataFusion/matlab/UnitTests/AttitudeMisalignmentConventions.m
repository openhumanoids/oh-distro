
% Check misalignment conventions

clc
clear

% standard system parameters
gn = [0;0;1];

% We have positive orientation errors in the local frame
pbPb = -[0.02;0;0];

tlQb = [1;0;0;0];
% tlQb = [0;0;0;1];
tlRb = q2R(tlQb)

dlQl = e2q(pbPb)
dlQl2 = [1; 0.5*pbPb]
dlRl = vec2skew(pbPb);


elQb = qprod(dlQl, tlQb);
% elQb = qprod(tlQb, dlQl);

elRb = (eye(3) + dlRl)*tlRb;

hat_gb = qrot(elQb, gn)
hat_gb = elRb*gn







