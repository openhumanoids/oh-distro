function [r,v,traj] = runSandiaPassive()

r= createSandiaManip();
v=r.constructVisualizer();
v.display_dt = 0.01;
x0=zeros(24,1);
x0(1)=-.2;
x0(2)=.2;
x0(5)=.2;
x0(8)=.2;
x0(11)=.2;
x0(4)=-.2;
x0(7)=-.2;
x0(10)=-.2;
traj = simulate(r,[0 3],x0);
playback(v,traj);

end