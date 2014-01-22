
% test qprod
clc

randomQs = 100;

% some basic functions we need
normQ = @(q) q/norm(q);
genQ = @() normQ(rand(4,1)-0.5); 


% generate a test header file

disp(['int randomQs;']);
disp(['randomQs = ' num2str(randomQs) ';'])
disp('Qa.resize(randomQs);');
disp('Qb.resize(randomQs);');
disp('Qc.resize(randomQs);');
disp ' '

for k = 0:(randomQs-1)
    Qa = genQ();
    Qb = genQ();
    Qc = qprod(Qb,Qa);
    
    disp(['Qa[' num2str(k) '].w() = ' num2str(Qa(1)) ';'])
    disp(['Qa[' num2str(k) '].x() = ' num2str(Qa(2)) ';'])
    disp(['Qa[' num2str(k) '].y() = ' num2str(Qa(3)) ';'])
    disp(['Qa[' num2str(k) '].z() = ' num2str(Qa(4)) ';'])
    
    disp(['Qb[' num2str(k) '].w() = ' num2str(Qb(1)) ';'])
    disp(['Qb[' num2str(k) '].x() = ' num2str(Qb(2)) ';'])
    disp(['Qb[' num2str(k) '].y() = ' num2str(Qb(3)) ';'])
    disp(['Qb[' num2str(k) '].z() = ' num2str(Qb(4)) ';'])
    
    disp(['Qc[' num2str(k) '].w() = ' num2str(Qc(1)) ';'])
    disp(['Qc[' num2str(k) '].x() = ' num2str(Qc(2)) ';'])
    disp(['Qc[' num2str(k) '].y() = ' num2str(Qc(3)) ';'])
    disp(['Qc[' num2str(k) '].z() = ' num2str(Qc(4)) ';'])


end