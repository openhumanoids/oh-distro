
% Check misalignment conventions

clc
clear

% standard system parameters
gn = [0;0;1];

% We have positive orientation errors in the local frame
p = [0.1; 0.2; 0.3];
p = [0.1;0;0];

tlQb = [1;0;0;0];
% tlQb = [0;0;0;1];

dlQl = e2q(p);

elQb = qprod(dlQl, tlQb);
% elQb = qprod(tlQb, dlQl);


hat_gb = qrot(elQb, gn)




