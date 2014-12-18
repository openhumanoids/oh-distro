function interactiveRecoveryTest()
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

x = [0;0];
xd = [0.5;0];
target = 0;
planning_time = 0;
options = struct('n_steps', 4);
xlimp = [x;xd];
[footsteps,result] = recoveryStepsFromLIMP(biped, xlimp, omega_0, foot_state, contact_state, options);
h1 = figure(1);

function OnClick(h, evt)
  ax = gca;
  c = get(ax, 'CurrentPoint');
  c = c(1,1:2)';
  if norm(c-x) < norm(c-xd)
    target = 1;
  else
    target = 2;
  end
end

function OnRelease(h, evt)
  target = 0;
end

function OnMove(h, evt)
  % ax = get(h, 'Parent');
  if target
    ax = gca;
    c = get(ax, 'CurrentPoint');
    c = c(1,1:2)';
    if target == 1
      xd = xd + c - x;
      x = c;
    elseif target == 2
      xd = c;
    end
    tic
    xlimp = [x; xd];
    [footsteps,result] = recoveryStepsFromLIMP(biped, xlimp, omega_0, foot_state, contact_state, options);
    planning_time = toc;
  end
end

set(h1, 'WindowButtonDownFcn', @OnClick);
set(h1, 'WindowButtonMotionFcn', @OnMove);
set(h1, 'WindowButtonUpFcn', @OnRelease);

while 1
  clf
  hold on
  plot(x(1), x(2), 'kx')
  plot(xd(1), xd(2), 'r^')
  plot([x(1), xd(1)], [x(2), xd(2)], 'k-')


  if isfield(result, 'x')
    rpos = [footsteps([footsteps.is_right_foot]).pos];
    lpos = [footsteps(~[footsteps.is_right_foot]).pos];
    rzmp = [footsteps([footsteps.is_right_foot]).zmp];
    lzmp = [footsteps(~[footsteps.is_right_foot]).zmp];
    plot(rpos(1,:), rpos(2,:), 'bx:')
    plot(lpos(1,:), lpos(2,:), 'bx:')
    plot(rzmp(1,:), rzmp(2,:), 'bo')
    plot(lzmp(1,:), lzmp(2,:), 'bo')
    
%     plot(foot_state.right(1), foot_state.right(2), 'bx');
%     plot(foot_state.left(1), foot_state.left(2), 'bx');
%     plot(result.footxy(1,1:2:end), result.footxy(2,1:2:end), 'bx:')
%     plot(result.footxy(1,2:2:end), result.footxy(2,2:2:end), 'bx:')
%     plot(result.COP(1,:), result.COP(2,:), 'bo')
    if result.caught
      plot(result.IC(1,:), result.IC(2,:), 'g*-')
    else
      plot(result.IC(1,:), result.IC(2,:), 'r*-')
    end
  end
  axis equal
  xlim([-1,1])
  ylim([-1,1])
  title(sprintf('Planning Time: %f ms', planning_time*1000))
  xlabel('$x$','Interpreter','LaTeX','FontSize',20)
  ylabel('$y$','Interpreter','LaTeX','FontSize',20)
  drawnow
end

end
