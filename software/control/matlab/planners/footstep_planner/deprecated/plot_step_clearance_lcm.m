function plot_step_clearance(biped, footpos)

id = 70;
  for f = {'left', 'right'}
    foot = f{1};
    for g = {'center', 'toe', 'heel'}
      grp = g{1};
      pts = zeros(3, length(footpos.(foot).orig(1,:)));
      for j = 1:length(pts(1,:))
        pos = biped.footOrig2Contact(footpos.(foot).orig(:,j), grp, strcmp(foot, 'right'));
        pts(:, j) = pos(1:3);
      end
      plot_lcm_points(pts', repmat([1;0;1], 1, length(pts(1,:)))', id, ['Step Clearance: ', foot, ' ', grp], 2, true);
      id = id + 1;
    end
  end



