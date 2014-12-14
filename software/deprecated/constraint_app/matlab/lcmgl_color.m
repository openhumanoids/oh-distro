function lcmgl_color(lcmgl, color)

    if ~ischar(color)
        c = 'rgbmcyy';
        ci = mod(color-1, length(c))+1;
        color = c(ci);
    end

    switch ( color ) 
        case 'r'
            lcmgl.glColor4f(1,0,0,1);
        case 'g'
            lcmgl.glColor4f(0,1,0,1);
        case 'b'
            lcmgl.glColor4f(0,0,1,1);
        case 'c'
            lcmgl.glColor4f(0,1,1,1);
        case 'm'
            lcmgl.glColor4f(1,0,1,1);
        case 'y'
            lcmgl.glColor4f(1,1,0,1);
        case 'k'
            lcmgl.glColor4f(0,0,0,1);
        case 'w'
            lcmgl.glColor4f(1,1,1,1);
    end