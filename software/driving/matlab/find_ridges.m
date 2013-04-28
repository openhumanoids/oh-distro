function out = find_ridges(in)

[gx,gy] = gradient(in);
f = fspecial('gaussian',[9,1],1);
gxx = myconv2(f,f,gx.*gx);
gyy = myconv2(f,f,gy.*gy);
gxy = myconv2(f,f,gx.*gy);
lambda1 = ((gxx+gyy) + sqrt((gxx-gyy).^2 + 4*gxy.^2))/2;
lambda2 = ((gxx+gyy) - sqrt((gxx-gyy).^2 + 4*gxy.^2))/2;
vx = ones(size(lambda1));
vy = gxy./(lambda1-gyy);
gx = vx;
gy = vy;
%vx=1, then vy = hxy/(lambda1-hyy)
%vy=1, then vx = hxy/(lambda1-hxx)
gmag = in;

%gmag = hypot(gx,gy);
gmag([1,end],:) = 0;
gmag(:,[1,end]) = 0;
msk = gmag>0;

[x,y] = meshgrid(1:size(in,2),1:size(in,1));
msk_x = abs(gx)>=abs(gy);
msk_y = ~msk_x;
msk_x = msk_x & msk;
msk_y = msk_y & msk;

xx = x(msk_x);
yy = y(msk_x);
yy_pos = yy+gy(msk_x)./gx(msk_x);
yy_frac = yy_pos-yy;
signs = sign(yy_frac);
yy_frac = abs(yy_frac);
gmag_pos = gmag(sub2ind(size(gmag),yy,xx+1)).*(1-yy_frac) + gmag(sub2ind(size(gmag),yy+signs,xx+1)).*yy_frac;
gmag_neg = gmag(sub2ind(size(gmag),yy,xx-1)).*(1-yy_frac) + gmag(sub2ind(size(gmag),yy-signs,xx-1)).*yy_frac;
gmag_mid = gmag(msk_x);
good = gmag_mid>gmag_neg & gmag_mid>gmag_pos;
ridges_x = [xx(good),yy(good)];

xx = x(msk_y);
yy = y(msk_y);
xx_pos = xx+gx(msk_y)./gy(msk_y);
xx_frac = xx_pos-xx;
signs = sign(xx_frac);
xx_frac = abs(xx_frac);
gmag_pos = gmag(sub2ind(size(gmag),yy+1,xx)).*(1-xx_frac) + gmag(sub2ind(size(gmag),yy+1,xx+signs)).*xx_frac;
gmag_neg = gmag(sub2ind(size(gmag),yy-1,xx)).*(1-xx_frac) + gmag(sub2ind(size(gmag),yy-1,xx-signs)).*xx_frac;
gmag_mid = gmag(msk_y);
good = gmag_mid>gmag_neg & gmag_mid>gmag_pos;
ridges_y = [xx(good),yy(good)];

ridges = [ridges_x;ridges_y];
out = false(size(in));
out(sub2ind(size(out),ridges(:,2),ridges(:,1))) = true;
out = bwmorph(out,'thin');
