foot_contact = struct('right', false, 'left', true);
foot_vertices = struct('right', [-0.05, 0.05, 0.05, -0.05; 
                                 -0.02, -0.02, 0.02, 0.02],...
                       'left', [-0.05, 0.05, 0.05, -0.05; 
                                 -0.02, -0.02, 0.02, 0.02]);
foot_state = struct('right', struct('position', [0;-0.13;0.1;0;0;0],...
                                    'velocity', [0.15;0;0;0;0;0]),...
                    'left', struct('position', [0; 0.13; 0.0811; 0;0;0],...
                                   'velocity', [0;0;0;0;0;0]));

com = [0;0];
comd = [0.15; -0.1];
omega = sqrt(9.81 / 0.84);

r_ic = com(1:2) + comd(1:2) / omega;

intercept_plans = QPReactiveRecoveryPlan.enumerateCaptureOptions(foot_contact, foot_vertices, foot_state, r_ic, 10, omega);
best_plan = QPReactiveRecoveryPlan.chooseBestIntercept(intercept_plans);
pp = QPReactiveRecoveryPlan.swingTraj(best_plan, foot_state.(best_plan.foot));

