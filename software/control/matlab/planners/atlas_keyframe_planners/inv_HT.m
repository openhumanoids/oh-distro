function T_out=inv_HT(T)
    M = T(1:3,1:3);
    p = T(1:3,4);
    T_out = zeros(4);
    T_out(1:3,1:3) = M';
    p= -M'*p;
    T_out(1:4,4) = [p(:); 1]; 
end