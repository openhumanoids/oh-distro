function pinv_A = pinv_gunter(A, param)    
    [u,s,v] = svd(A);
    pinv_A = v*diaginv_gunter(s, param)*u';