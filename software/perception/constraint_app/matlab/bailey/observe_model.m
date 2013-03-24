function z = observe_model(x, idf)

Nxv= 3; % number of vehicle pose states
fpos= Nxv + idf*2 - 1; % position of xf in state

% auxiliary values
dx= x(fpos,:)  -x(1,:); 
dy= x(fpos+1,:)-x(2,:);
d2= dx.^2 + dy.^2;
d= sqrt(d2);

% predict z
z= [d;
    atan2(dy,dx) - x(3,:)];
