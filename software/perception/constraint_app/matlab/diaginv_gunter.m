function A = diaginv_gunter(B, threshold)
    A = B';
    mindim = min(size(A));
    d = sub2ind(size(A), 1:mindim, 1:mindim);
    m = A(d) < threshold;
    A(d(~m)) = 1./A(d(~m)) ;
    A(d(m)) = 1/threshold^2 .* A(d(m));