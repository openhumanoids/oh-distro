f = fdesign.lowpass('Fp,Fst,Ap,Ast',.4,.6,1,80);
Hd1 = design(f,'equiripple');


hfvt = fvtool(Hd1,'Color','White');
