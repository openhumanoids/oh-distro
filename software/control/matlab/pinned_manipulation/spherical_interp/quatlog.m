function val = quatlog(q)
w = q(1);
v= q(2:4);
if(norm(v)>eps)
 val =[log(norm(q)) (v/norm(v))*acos(w/norm(q))];
else
 val= [log(norm(q)) 0*v*acos(w/norm(q))];
end

end