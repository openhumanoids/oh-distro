function p = singular_mvnpdf(d, sqrt_precision)
    Ld = sqrt_precision * d';
    p = exp ( -0.5 * sum(Ld.^2,1) )';