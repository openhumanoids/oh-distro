function obj = new_spline_warp(n,max_in)

obj.n = n;
obj.scale_factor = n/max_in;
obj.func = @warp_func;
obj.set_params = @set_params;
obj.params = zeros(2*n);


function obj = set_params(obj, v)

obj.params = v;

k = 1/obj.scale_factor;
cc = zeros(obj.n+1,4);
cc(1,1) = v(2) + k - 2*v(1);
cc(1,2) = 3*v(1) - 2*k - v(2);
cc(1,3) = k;
cc(1,4) = 0;
for i = 1:obj.n-1
    cc(i+1,1) = v(2*i+2) + v(2*i) - 2*v(2*i+1) + 2*v(2*i-1);
    cc(i+1,2) = 3*v(2*i+1) - 3*v(2*i-1) - 2*v(2*i) - v(2*i+2);
    cc(i+1,3) = v(2*i);
    cc(i+1,4) = v(2*i-1);
end
cc(end,1) = 0;
cc(end,2) = 0;
cc(end,3) = v(end);
cc(end,4) = v(end-1);
obj.coeffs = cc;



function out = warp_func(obj,in)

in = abs(in(:));
scaled_in = in*obj.scale_factor;
int_in = floor(scaled_in);
which_spline = int_in+1;
which_spline(which_spline>size(obj.coeffs,1)) = size(obj.coeffs,1);
x = scaled_in-which_spline+1;
c = obj.coeffs(which_spline,:);

%out = c(:,1).*x.^3 + c(:,2).*x.^2 + c(:,3).*x + c(:,4);
out = c(:,3).*x + c(:,4);
xx = x.*x;
out = out + c(:,2).*xx;
xx = xx.*x;
out = out + c(:,1).*xx;
