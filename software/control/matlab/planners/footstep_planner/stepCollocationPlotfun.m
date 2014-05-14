function stop = stepCollocationPlotfun(x, r_ndx, l_ndx)
  [steps, steps_rel] = decodeCollocationSteps(x);
  steps
  steps_rel
  clf
  quiver3(steps(1,r_ndx), steps(2,r_ndx), steps(3,r_ndx), cos(steps(6,r_ndx)), sin(steps(6,r_ndx)), zeros(size(steps(1,r_ndx))), 0.5,'go','ShowArrowHead','on');
  hold on
  plot3(steps(1,r_ndx), steps(2,r_ndx), steps(3,r_ndx), 'g:');
  quiver3(steps(1,l_ndx), steps(2,l_ndx), steps(3,l_ndx), cos(steps(6,l_ndx)), sin(steps(6,l_ndx)), zeros(size(steps(1,l_ndx))), 0.5,'ro','ShowArrowHead','on');
  plot3(steps(1,l_ndx), steps(2,l_ndx), steps(3,l_ndx),'r:');
  hold off
  axis equal
%   drawnow();
  stop = false;
end

