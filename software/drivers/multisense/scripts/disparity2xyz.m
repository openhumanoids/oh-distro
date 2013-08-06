
Q = [1, 0, 0, -512.5;
  0, 1, 0, -272.5;
  0, 0, 0,  606.0344848632812;
  0, 0, 1/.07, 0] ;

x = 212;
y = 400;
z = 36.3125;
w = 1;
out =Q*([x y z w]');
Wscale = out(4);
scaled = out./Wscale



Wscale = Q(4,3)*z;
simple = [ ( Q(1,1)*x + Q(1,4)*w )/Wscale ...
  ( Q(2,2)*y + Q(2,4)*w )/Wscale ...
  ( Q(3,4)*w )/Wscale ]