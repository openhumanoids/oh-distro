function val = quatexp(q)
w = q(1);
v= q(2:4);
if(norm(v)>eps)
 val = exp(w)*[cos(norm(v)) (v/norm(v))*sin(norm(v))];
else
 val= exp(w)*[cos(norm(v)) 0*v*sin(norm(v))];
end

end