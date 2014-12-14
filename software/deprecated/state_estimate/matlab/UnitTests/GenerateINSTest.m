

clc

genE = @() randn(3,1);

INSpose = init_pose();
INSpose__k1 = init_pose();
INSpose__k2 = init_pose();

inertialData = init_inertialData(9.8);


iter = 100;


dt = 0.01;
dE = [];
LQB = [];
de_ = [0;0;0];

for k = 1:iter
    
    de = 0.01*genE() + de_;
    dE = [dE; de'];
    
    % generate IMU data measurement frame
    inertialData.predicted.utime = k*dt*1E6;
    inertialData.predicted.w_b = de/dt;


    INSpose = INS_lQb([], INSpose__k1, INSpose__k2, inertialData);

    INSpose__k2 = INSpose__k1;
    INSpose__k1 = INSpose;
    
    LQB = [LQB; INSpose.lQb'];
    
    de_ = de;

end


disp(['int iter;']);
disp(['iter = ' num2str(iter) ';'])
disp('dE.resize(iter);');
disp('LQB.resize(iter);');
disp ' '

for k=0:(iter-1)
    
    disp(['dE[' num2str(k) '](0) = ' num2str(dE(k+1,1)) ';'])
    disp(['dE[' num2str(k) '](1) = ' num2str(dE(k+1,2)) ';'])
    disp(['dE[' num2str(k) '](2) = ' num2str(dE(k+1,3)) ';'])
    
    disp(['LQB[' num2str(k) '].w() = ' num2str(LQB(k+1,1)) ';'])
    disp(['LQB[' num2str(k) '].x() = ' num2str(LQB(k+1,2)) ';'])
    disp(['LQB[' num2str(k) '].y() = ' num2str(LQB(k+1,3)) ';'])
    disp(['LQB[' num2str(k) '].z() = ' num2str(LQB(k+1,4)) ';'])
    
end

plot(LQB)
sum(abs(1-sqrt(sum(LQB.^2,2))))
