options.floating = true;
options.dt = 0.001;
biped = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),options);
biped = removeCollisionGroupsExcept(biped,{'heel','toe'});
biped = compile(biped);

foot_state.right = [0;-0.15;0;0;0;0];
foot_state.left = [0;0.15;0;0;0;0];
contact_state.right_contact = true;
contact_state.left_contact = true;

omega_0 = sqrt(9.81 / 1);

n = 40;
x = linspace(-.6, .6, n);
y = linspace(-.3, .3, n);
[X, Y] = meshgrid(x, y);
N = zeros(size(X));

options = struct('n_steps', 7);
for j = 1:n
  xi = x(j);
  for k = 1:n
    yi = y(k);
    xlimp = [xi;yi;0;0];
    % xlimp = [0;xi;0;xdi];
    [~, result] = recoveryStepsFromLIMP(biped, xlimp, omega_0, foot_state, contact_state, options);
    if result.caught
      N(k,j) = size(result.footxy, 2)-1;
    else
      N(k,j) = nan;
    end
    assert (X(k,j) == xi);
    assert (Y(k,j) == yi);
  end
end


colormap('hot');
contourf(X, Y, N, (1:5)-0.5); 
caxis([0,5]);
c = colorbar('YLim', [0,5],'YTick',(1:5)-0.5, 'YTickLabel',1:100,'FontSize',20);
xlabel('$x_{IC0}$','Interpreter','LaTeX','FontSize',20)
ylabel('$y_{IC0}$','Interpreter','LaTeX','FontSize',20)
