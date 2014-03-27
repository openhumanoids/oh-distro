function [skew] = vec2skew(v)
% Function to convert a 3x1 to 3x3 skew matrix
% Version: 0.1 -14/6/2012, dimension 3 only

if (max(size(v))~=3)
    skew = [];
    return
end
if (min(size(v))~=1)
    skew = [];
    return
end


skew = [0, -v(3), v(2);...
        v(3), 0, -v(1);...
        -v(2), v(1), 0];
    
    