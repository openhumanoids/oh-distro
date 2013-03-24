function xv= vehicle_model(x, V,G, WB,dt)

xv= [x(1,:) + V.*dt.*cos(G+x(3,:)); 
     x(2,:) + V.*dt.*sin(G+x(3,:));
     x(3,:) + V.*dt.*sin(G)/WB];
 
xv(3,:) = pi_to_pi(xv(3,:));
