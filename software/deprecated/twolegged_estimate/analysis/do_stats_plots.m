function do_stats_plots(results)

stats = [results.stats];
vels = [stats.vel];
names = {results.name};

figure(10);
clf;
title('rms error');
ylabel('rms error');
title('rms error');
vals = cat(1,vels.rms_error);
plot(vals);
grid on;
xticklabel_rotate(1:numel(names), 90, names, 'interpreter','none');

figure(11);
clf;
title('max error');
ylabel('max error');
title('max error');
vals = cat(1,vels.max_error);
plot(vals);
grid on;
xticklabel_rotate(1:numel(names), 90, names, 'interpreter','none');

figure(12);
clf;
title('num exceeding');
ylabel('num exceeding');
title('num exceeding');
vals = cat(1,vels.pct_exceeding);
plot(vals);
grid on;
xticklabel_rotate(1:numel(names), 90, names, 'interpreter','none');

