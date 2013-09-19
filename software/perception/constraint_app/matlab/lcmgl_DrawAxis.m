function lcmgl_DrawAxis(lcmgl, T, scale, cx, cy, cz, varargin)  

    linewidth = 3;

    optargin = size(varargin,2);
    if optargin >= 1
        linewidth = varargin{1};
    end

    origin = T * [0; 0; 0; 1];
    xaxis = T * [scale; 0; 0; 1];
    yaxis = T * [0; scale; 0; 1];
    zaxis = T * [0; 0; scale; 1];

    h = zeros(1,3);
    
    a = [origin xaxis]';
    lcmgl_line(lcmgl, a, cx, linewidth);
    a = [origin yaxis]';
    lcmgl_line(lcmgl, a, cy, linewidth);
    a = [origin zaxis]';
    lcmgl_line(lcmgl, a, cz, linewidth);
