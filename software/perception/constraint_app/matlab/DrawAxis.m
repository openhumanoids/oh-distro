function h = DrawAxis(T, scale, cx, cy, cz, varargin)  

    linewidth = 3;
    linestyle = '-';

    optargin = size(varargin,2);
    if optargin >= 1
        linewidth = varargin{1};
    end
    if optargin == 2
        linestyle = varargin{2};
    end

    origin = T * [0; 0; 0; 1];
    xaxis = T * [scale; 0; 0; 1];
    yaxis = T * [0; scale; 0; 1];
    zaxis = T * [0; 0; scale; 1];

    h = zeros(1,3);
    
    a = [origin xaxis];
    h(1)=line(a(1,:), a(2,:), a(3,:), 'Color', cx, 'LineWidth', linewidth, 'LineStyle', linestyle);
    a = [origin yaxis];
    h(2)=line(a(1,:), a(2,:), a(3,:), 'Color', cy, 'LineWidth', linewidth, 'LineStyle', linestyle);
    a = [origin zaxis];
    h(3)=line(a(1,:), a(2,:), a(3,:), 'Color', cz, 'LineWidth', linewidth, 'LineStyle', linestyle);
