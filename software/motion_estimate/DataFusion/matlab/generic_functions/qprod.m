function q = qprod(b,c)
% quaternions are scalar vector

B = [b(1), -b(2), -b(3), -b(4);...
     b(2),  b(1), -b(4),  b(3);...
     b(3),  b(4),  b(1), -b(2);...
     b(4), -b(3),  b(2),  b(1)];
 
 q = B*c;
 