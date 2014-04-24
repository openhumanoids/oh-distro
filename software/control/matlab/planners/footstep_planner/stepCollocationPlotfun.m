function stop = stepCollocationPlotfun(x, r_ndx, l_ndx, terrain, corridor_pts)
  [steps, steps_rel] = decodeCollocationSteps(x);
  steps
  steps_rel
  clf
  quiver3(steps(1,r_ndx), steps(2,r_ndx), steps(3,r_ndx), cos(steps(6,r_ndx)), sin(steps(6,r_ndx)), zeros(size(steps(1,r_ndx))), 0.5,'go','ShowArrowHead','on');
  hold on
  plot3(steps(1,r_ndx), steps(2,r_ndx), steps(3,r_ndx), 'g:');
  quiver3(steps(1,l_ndx), steps(2,l_ndx), steps(3,l_ndx), cos(steps(6,l_ndx)), sin(steps(6,l_ndx)), zeros(size(steps(1,l_ndx))), 0.5,'ro','ShowArrowHead','on');
  plot3(steps(1,l_ndx), steps(2,l_ndx), steps(3,l_ndx),'r:');
  [X, Y] = meshgrid(linspace(-0.5, 1), linspace(-2, 0));
  heights = terrain.getHeight([reshape(X, 1, []); reshape(Y, 1, [])]);
  mesh(X, Y, reshape(heights, size(X)));
  plot3(corridor_pts(1,[1:end, 1]), corridor_pts(2,[1:end, 1]), terrain.getHeight(corridor_pts(:,[1:end,1])))
  hold off
  axis equal
%   drawnow();
  stop = false;
end

