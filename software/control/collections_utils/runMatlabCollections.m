% demonstration of Collections to LCM

% 2. Plot a set of axes:
pos= rand(10,3) 
pos(:,1) = pos(:,1) - 2
cols=repmat( [1,0,0], size(pos))
plot_lcm(pos(:,1), pos(:,2), pos(:,3), 9, 'Test', 5, 1)

% 2. Plot a set of points and lines
pos= rand(10,3)
cols=repmat( [1,0,0], size(pos))

plot_lcm_points(pos, cols, 10, 'Test Points', 1, 1)
plot_lcm_points(pos, cols, 11, 'Test Line', 2, 1)


% 3. Plot an image:
mit=imread('mit_logo.png');
image(mit)
x = repmat(1:32, 32,1);
y = repmat(1:32, 1,32);
z = rand(32,32);
pos = [x(:), y(:), z(:)];
pos(:,1) = -pos(:,1);
cols = reshape( double(mit)/255 ,32*32,3);
plot_lcm_points(pos, cols, 12, 'Earth', 1, 1)