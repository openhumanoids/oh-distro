file ='heightmap.txt'
d=load(file)

%imagesc(d,[-2,4])
%colorbar

[dx,dy]= gradient(d)
dxy= 4*sqrt(dx.*dx + dy.*dy)

figure
imagesc(dxy,[0 1])
colorbar

dlmwrite('gradmap.txt', dxy, 'delimiter', ' ', ...
         'precision', 6)