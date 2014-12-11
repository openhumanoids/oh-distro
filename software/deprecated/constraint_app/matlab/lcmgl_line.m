function lcmgl_line(lcmgl, p, color, width)
    GL_POINTS = 0;
    GL_LINES = 1;
    GL_LINE_LOOP = 2;
    GL_DEPTH_TEST = 2929;
    
    lcmgl.glEnable(GL_DEPTH_TEST);
    
    lcmgl_color(lcmgl, color);

    lcmgl.glLineWidth(width);
    
    lcmgl.glBegin(GL_LINE_LOOP);
    for i = 1:size(p,1)
        lcmgl.glVertex3f(p(i,1), p(i,2), p(i,3));
    end
    lcmgl.glEnd();