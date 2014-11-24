%d = load('/home/drc/Desktop/latency_benchmarks/driver_tics_00.txt');
%d = load('/home/drc/Desktop/latency_benchmarks/driver_tics_01.txt');
%d = load('/home/drc/Desktop/latency_benchmarks/driver_tics_02.txt');
%d = load('/home/drc/Desktop/latency_benchmarks/driver_tics_03.txt');

d = load('/home/drc/Desktop/latency_benchmarks/driver_tics_05b.txt');
%d = load('/home/drc/Desktop/latency_benchmarks/driver_tics_05c.txt');

%d = load('/tmp/driver_tics.txt');

figure
subplot(2,1,1)

t = (d(:,1) - d(1,1))*1e-6

plot( t, d(:,3) ,'.')
subplot(2,1,2)
hist(d(:,3),[500:10:3500])

mean(d(:,3))
median(d(:,3))