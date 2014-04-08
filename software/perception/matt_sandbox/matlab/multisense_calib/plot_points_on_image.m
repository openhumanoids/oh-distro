function plot_points_on_image(pts,K,img)

cmap = colormap('jet');
z = pts(:,3);
pix = pts*K';
pix = pix(:,1:2)./pix(:,[3,3]);
colormap_inds = round((z-min(z))/(max(z)-min(z))*(size(cmap,1)-1))+1;
dat = sortrows([pix,colormap_inds],3);
starts = [1;find(diff(dat(:,3))>0)+1];
ends = [starts(2:end)-1;size(dat,1)];
figure, imshow(img);
hold on;
for i = numel(starts):-1:1
    pix_sub = dat(starts(i):ends(i),1:2);
    myplot(pix_sub,'.','color',cmap(i,:));
end
hold off;
