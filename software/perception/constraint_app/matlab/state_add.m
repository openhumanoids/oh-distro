function s = state_add(a,b)    
    % s = a + b; 
    % a is [eulder, joints] and b is [delta_euler, delta_jointsl]
    s = a + b;