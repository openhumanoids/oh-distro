function lens = new_spherical_dist(param)

a = param^4;
lens.dist.a = a;
lens.dist.func = @spherical_distort;

lens.undist.a = a;
lens.undist.func = @spherical_undistort;



function pix = spherical_distort(obj,rays)

pts = rays(:,1:2)./rays(:,[3,3]);
r_u_2 = pts(:,1).^2 + pts(:,2).^2;
r_d_2 = r_u_2./sqrt(1 + obj.a*r_u_2);
ratio = r_d_2./(r_u_2+1e-10);
pix = pts.*[ratio,ratio];


function rays = spherical_undistort(obj,pix)

r_d_2 = pix(:,1).^2 + pix(:,2).^2;
r_u_2 = r_d_2./sqrt(1 - obj.a*r_d_2);
ratio = r_u_2./(r_d_2+1e-10);
rays = pix.*[ratio,ratio];
rays = [rays,ones(size(rays,1),1)];
rays = rays./repmat(sqrt(sum(rays.^2,2)),[1,3]);
