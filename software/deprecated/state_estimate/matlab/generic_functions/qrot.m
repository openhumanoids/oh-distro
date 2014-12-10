function v_b = qrot(aQb,v_a)
% rotate a three vector v from frame a to b using aQb quaternion
% quaternions are scalar vector: [w;x;y;z]

V_A = [0;v_a];

V_B = qprod( aQb,  qprod(V_A,qconj(aQb))  );

v_b = V_B(2:4);
