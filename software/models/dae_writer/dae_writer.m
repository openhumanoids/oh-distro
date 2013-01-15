function dae_writer()
%s=single_tri()
%s=random_tri();

output_path = '../mit_gazebo_models/simple_plane/meshes/';
s=terrain_simulator(output_path);

s=verts2output(s);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fname = [ output_path 'simple_plane.dae'];
%fname = '../mit_gazebo_models/diy_box/meshes/diy_box.dae';


fid_out = fopen(fname,'w');
tag_top=readf('snippets/tag_top.dae',fid_out);
tag0=readf('snippets/tag0.dae',fid_out);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tag1a=readf('snippets/tag1a.dae',fid_out)  ;
fprintf(fid_out,'%d',prod(size(s.vert)));
tag1b=readf('snippets/tag1b.dae',fid_out)    ;
fprintf(fid_out,'%s',  sprintf('%f ',s.vert)    );
tag1c=readf('snippets/tag1c.dae',fid_out)   ;
fprintf(fid_out,'%d',prod(size(s.vert,1)));
tag1d=readf('snippets/tag1d.dae',fid_out)    ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tag2a=readf('snippets/tag2a.dae',fid_out)  ;
fprintf(fid_out,'%d',prod(size(s.norm)));
tag2b=readf('snippets/tag2b.dae',fid_out)  ;
fprintf(fid_out,'%s',  sprintf('%f ',s.norm)    );
tag2c=readf('snippets/tag2c.dae',fid_out)    ;
fprintf(fid_out,'%d',size(s.norm,1));
tag2d=readf('snippets/tag2d.dae',fid_out)    ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tag3a=readf('snippets/tag3a.dae',fid_out)   ;
fprintf(fid_out,'%d',prod(size(s.text)));
tag3b=readf('snippets/tag3b.dae',fid_out) ;
fprintf(fid_out,'%s',  sprintf('%f ',s.text)    );
tag3c=readf('snippets/tag3c.dae',fid_out)  ;
fprintf(fid_out,'%d', size(s.text,1) );
tag3d=readf('snippets/tag3d.dae',fid_out)   ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tag4a=readf('snippets/tag4a.dae',fid_out)    ;
fprintf(fid_out,'%d', size(s.Wall,1) );
tag4b=readf('snippets/tag4b.dae',fid_out) ;
fprintf(fid_out,'%s',  sprintf('%d ',s.Wall)    );
tag4c=readf('snippets/tag4c.dae',fid_out)  ;


tag5=readf('snippets/tag5.dae',fid_out)  ;
tag6=readf('snippets/tag6.dae',fid_out)  ;

fclose(fid_out);
disp('file written | done')

% num of meshes
% x.Children(10).Children(2).Children(2).Children(2).Children(2).Attributes(1).Value

% mesh list Mesh-Geometry-Position-array
% x.Children(10).Children(2).Children(2).Children(2).Children(2).Children.Data


function s=random_tri()
n_vert = 100;
for i=1:n_vert
    s.vert_list{i} =rand(3,3)
end

% 1st pos forward
% 2nd pos left
% 3rd pos up

function s=single_tri()

s.vert_list{1} =[1.0 1.0 1.0;
    1.0 1.0 3.0;
    1.0 2.0 1.0];

%s.vert_list{2} =[0.0 0.0 0.0;
%    0.0 1.0 0.0;
%    0.0 0.0 8.0];


function s=verts2output(s)
disp('verts2output')
s.vert=[];
for i=1:size(s.vert_list,2)
    s.vert =[s.vert ; s.vert_list{i}];
end
s.vert = s.vert' ;


s.norm = [0.00000 0.00000 -1.00000];
s.text=[];
for i=1:size(s.text_list,2)
    s.text =[s.text ; s.text_list{i}];
end
s.text = s.text' ;

% same texture for each triangle:
%one_text = [0.0 0.0;
%    0.0 1.0;
%    1.0 1.0];
%s.text =  repmat(one_text,  size(s.vert,2)/3  ,1)';

s.Wvert =[ 0:  size(s.vert,2)-1];
s.Wnorm = zeros(size(s.vert,2),1)';
s.Wtext =[ 0:  size(s.text,2)-1];
%s.Wtext = repmat(  [ 0: 2]  , 1, size(s.vert,2)/3  );
temp = [s.Wvert;s.Wnorm;s.Wtext];
s.Wall = temp(:);



function fout=readf(fname,fid_out)
fout=[];
fid=fopen(fname);
while 1
    tline = fgetl(fid);
    if ~ischar(tline), break, end
    %fout= [fout;tline];
    
    fprintf(fid_out,'%s\n',tline);
    
end
fclose(fid);