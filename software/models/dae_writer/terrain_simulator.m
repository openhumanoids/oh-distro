function s=terrain_simulator(image_path)
close all


c=0;

% 1 = steps
% 2 = wedge
% 3 = image
% 4 = canyon
which=4
if (which==1)
    height=40;
    width=20;
    
    h=make_steps(width,height)
    s=convert_array_to_tri(h)
    rescale= [0.2,0.2,1]
elseif (which==2)
    height=400;
    width=100;
    
    c=c+1;
    s.vert_list{c} = [  [0  0  0];   [ height 0 4 ]     ;  [0 width  0 ]   ];
    c=c+1;
    s.vert_list{c} = [  [ height  0 4];   [  height width 4]     ;  [0 width 0 ]   ];
    rescale= [0.1,0.1,1];
elseif (which==3)
    x= imread('mit_terrain_257_v1.png');
    x= x(1:20:end,1:20:end,1); % decimate
    figure
    imagesc(x(:,:,1)) ;
    
    h =double(x(:,:,1));
    h=h+5;
    
    s=convert_array_to_tri(h)
    rescale= [1,1,0.01]
    height = size(h,1);
    width = size(h,2);
elseif (which==4)
    h=zeros(10,10);
    %keyboard
    h=[0     1     2     3     4     5     6     7     7     7
        0     1     2     3     4     5     6     7     7     7
        0     1     2     3     4     5     6     7     7     7
        0     0     0     0     0     0     0     8     8     8
        13    13    13    12    11    10    9     8     8     8
        13    13    13    12    11    10    9     8     8     8
        13    13    13    12    11    10    9     8     8     8
        14    14    14    0     0     0     0     0     0     0
        15    15    16    17    18    19    20    21    22    23
        15    15    16    17    18    19    20    21    22    23
        15    15    16    17    18    19    20    21    22    23]*20
    
    % maze:
    h=[400	400	400	400	400	400	400	400	400	400	400	400	400	400	400
        0	10	10	10	10	10	400	10	10	10	10	10	10	10	400
        0	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        0	10	10	400	10	10	10	10	10	400	10	10	10	10	400
        400	400	400	400	400	400	400	400	400	400	400	10	10	10	400
        400	10	10	10	10	10	10	400	10	10	10	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	400	10	10	10	10	10	400	10	10	10	400
        400	10	10	10	400	400	400	400	400	400	400	400	400	400	400
        400	10	10	10	10	10	10	10	400	10	10	10	400	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	10	400	10	10	10	10	10	10	10	10	400
        400	400	400	400	400	400	400	400	400	400	400	10	10	10	400
        400	10	10	10	10	10	10	400	10	10	10	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	400	10	10	10	10	10	400	10	10	10	400
        400	10	10	10	400	400	400	400	400	400	400	400	400	400	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	400	10	10	10	10	10	10	10	10	10	400
        400	400	400	400	400	400	400	400	400	400	400	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	400	400	400	400	400	400	400	400	400	400	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	10	10	10	10	10	10	10	10	10	10	10	10	10	400
        400	400	400	400	400	400	400	400	400	400	400	0	0	0	400];
         rescale= [4,4,0.02]
         
    % simpler - for steven:
%     h=[ 50	50	50	50	50	50	50	50	50	50	50	50	50	50	50
%         0	10	10	10	10	10	50	10	10	10	10	10	10	10	50
%         0	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         0	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         0	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         0	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         0	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         0	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	50	50	50	50	50	50	50	50	50	10	10	10	10	50
%         50	50	50	50	50	50	50	50	50	50	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	50	50	50	50	50	50	50	50	50	50	50
%         50	10	10	10	50	50	50	50	50	50	50	50	50	50	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	10	10	10	10	10	10	10	10	10	10	10	10	10	50
%         50	50	50	50	50	50	50	50	50	50	50	10	10	10	50
%         50	50	50	50	50	50	50	50	50	50	50	10	10	10	50
%         50	50	50	50	50	50	50	50	50	50	50	0	0	0	50];
%     rescale= [4,4,0.02]

%    h=[0	0
%        0	0];
%    rescale= [80,80,0.02];

    %keyboard
    s=convert_array_to_tri(h)
    height = size(h,1);
    width = size(h,2);
end

% add floor:
%c=size(s.vert_list,2)+1;
%s.vert_list{c} = [  [0  0  0];   [ height 0 0 ]     ;  [0 width  0 ]   ];
%c=size(s.vert_list,2)+1;
%s.vert_list{c} = [  [ height  0 0];   [  height width 0]     ;  [0 width 0 ]   ];


for i=1:size(s.vert_list,2)
    s.vert_list{i}(:,1) = s.vert_list{i}(:,1)*rescale(1);
    s.vert_list{i}(:,2) = s.vert_list{i}(:,2)*rescale(2);
    s.vert_list{i}(:,3) = s.vert_list{i}(:,3)*rescale(3);
end


disp('about to plot')
do_plot=1;
if do_plot==1
    figure
    surf(h')
    axis xy
    %    axis equal
    
    figure
    hold on
    for i=1:size(s.vert_list,2)
        tri1 = s.vert_list{i};
        fill3( tri1(:,1) ,  tri1(:,2)      ,  tri1(:,3) ,  max(tri1(:,3))  );
    end
end
disp('writing height map to image file')
imwrite((flipud(h'/ (max(h(:))+1) )), [image_path '/simple_box.jpg'])





function s=convert_array_to_tri(h)
s.vert_list=[];
s.text_list=[];
for i=1:(size(h,1) -1)  % height
    for j=1:(size(h,2) -1)  % width
        % adjustments to map texture into range 0->1
        top = 1;
        bottom = repmat( (size(h) -1) , 3,1);
        
        tri1 = [[i i+1 i];  [j j j+1]      ;  [h(i,j)     , h(i+1,j) , h(i,j+1)]]';
        c=size(s.vert_list,2)+1;
        s.vert_list{c} =tri1;
        s.text_list{c} = (tri1(:,1:2)-top) ./ bottom;
        
        tri2 = [  [i+1 i+1 i];  [j j+1 j+1]     ;  [h(i+1,j)     , h(i+1,j+1) , h(i,j+1)]   ]';
        c=size(s.vert_list,2)+1 ;
        s.vert_list{c} =tri2;
        s.text_list{c} =(tri2(:,1:2)-top) ./ bottom;
    end
end


function h=make_steps(width,height)
h=rand(height, width)/4

%h =wiener2(h,[4 4]);
h =wiener2(h,[4 4]);

step_height = 0.2

%add some footsteps:
foot_size =[2,1];
h=set_footstep(h, 2,2,step_height , foot_size);
h=set_footstep(h, 5,5,step_height , foot_size);
h=set_footstep(h, 7,2,step_height , foot_size);
h=set_footstep(h, 10,5,step_height , foot_size);
h=set_footstep(h, 13,2,step_height , foot_size);
h=set_footstep(h, 16,5,step_height , foot_size);
h=set_footstep(h, 19,2,step_height , foot_size);
h=set_footstep(h, 22,5,step_height , foot_size);
h=set_footstep(h, 25,2,step_height , foot_size);
h=set_footstep(h, 28,5,step_height , foot_size);

h=set_footstep(h, 34,2,step_height , foot_size);
h=set_footstep(h, 34,5,step_height , foot_size);
h=set_footstep(h, 34,4,step_height , foot_size);
h=set_footstep(h, 36,4,step_height , foot_size);
h=set_footstep(h, 36,2,step_height , foot_size);
h=set_footstep(h, 36,5,step_height , foot_size);
h=set_footstep(h, 37,4,step_height , foot_size);
h=set_footstep(h, 37,2,step_height , foot_size);
h=set_footstep(h, 37,5,step_height , foot_size);


foot_size =[1,2];
h=set_footstep(h, 35,8,step_height , foot_size);
h=set_footstep(h, 38,10,step_height , foot_size);
h=set_footstep(h, 35,12,step_height , foot_size);
h=set_footstep(h, 38,14,step_height , foot_size);
h=set_footstep(h, 35,16,step_height , foot_size);
h=set_footstep(h, 38,18,step_height , foot_size);

%h=set_footstep(h, 60,30,step_height , foot_size);
%h=set_footstep(h, 100,10,step_height , foot_size);
%h=set_footstep(h, 140,30,step_height , foot_size);
%h=set_footstep(h, 180,10,step_height , foot_size);
%h=set_footstep(h, 220,30,step_height , foot_size);
%h=set_footstep(h, 260,10,step_height , foot_size);
%h=set_footstep(h, 300,30,step_height , foot_size);
%h=set_footstep(h, 360,30,step_height , foot_size);
%h=set_footstep(h, 360,10,step_height , foot_size);

% do some smoothing
%h =medfilt2(h,[4 4])

% was using this:
%h =wiener2(h,[4 4]);
%h =wiener2(h,[4 4]);

%h =filter2([1,1;1,1],h)

function h=set_footstep(h ,i,j,val, foot_size )


%for i2=(i-p_size_x:i+p_size_x)
%    for j2=(j-p_size_y:j+p_size_y)


for i2=(i:i+foot_size(1) ) % x
    for j2=(j:j+ foot_size(2) ) %y
        h(i2,j2) = val + rand/120;
    end
end