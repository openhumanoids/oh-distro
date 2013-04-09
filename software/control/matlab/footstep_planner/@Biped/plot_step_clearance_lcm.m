function plot_step_clearance(biped, foottraj, num_steps)


  ts = linspace(foottraj.right.orig.tspan(1), foottraj.right.orig.tspan(end), num_steps * 10);

id = 70;
  for f = {'left', 'right'}
    foot = f{1};
    for g = {'center', 'toe', 'heel'}
      grp = g{1};
      pts = zeros(3, length(ts));
      for j = 1:length(ts)
        pos = biped.footOrig2Contact(foottraj.(foot).orig.eval(ts(j)), grp, strcmp(foot, 'right'));
        pts(:, j) = pos(1:3);
      end
      % HACK for Quals 1
      pts(3, :) = pts(3, :) + 1;
      plot_lcm_points(pts', repmat([1;0;1], 1, length(ts))', id, ['Step Clearance: ', foot, ' ', grp], 3, true);
      id = id + 1;
    end
  end



