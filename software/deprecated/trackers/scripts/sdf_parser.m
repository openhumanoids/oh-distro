function main()
%outputformat
%box,posexyzrpy,sizexyz
%cylinder,posexyzrpy,radius,length

close all
% 0 read from mat, 1 read from sdf
links = vehicle_reader(0)

strings ={};
for j=1%:size(links,1) % lnik number
  j
  link_strings = getLinkAsStrings(links(j));
  strings = [strings; link_strings(:)];
end

fid = fopen('chassis.txt','w');
for i=1:size( strings, 1)
  fprintf(fid,'%s\n',strings{i});
end
fclose(fid);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function strings = getLinkAsStrings(link)
for i=1:size( link.collisions, 1)
  pose = getPoseFromCollision(link.collisions(i) );
  
  %link.collisions(i).geometry.Children(2)
  for j=1:size(link.collisions(i).geometry.Children,2)
    geom_field = link.collisions(i).geometry.Children(j)
    if (strcmp (geom_field.Name , 'box' ) )
      col_size = getBox(geom_field);
      type ='box';
    elseif (strcmp (geom_field.Name , 'cylinder' ) )
      col_size = getCylinder(geom_field);
      type ='cylinder';
    else
      %keyboard
    end
  end
  string_out = [link.name ',' type  ',' pose ',' col_size];
  strings{i} = string_out;
end



function pose = getPoseFromCollision(collision)
if isfield(collision, {'pose'})
  x =collision.pose;
  pose = x.Children.Data;
  pose = strrep(pose,' ',',');
else
  pose = '0,0,0,0,0,0';
end

function box_size = getBox(geom_field)
for j=1:size(geom_field.Children,2)
  field = geom_field.Children(j)
  if (strcmp (field.Name , 'size' ) )
    box_size = field.Children.Data
  else
    % else freespace
    %keyboard
  end
end
box_size = strrep(box_size,' ',',');


function col_size = getCylinder(geom_field)

for j=1:size(geom_field.Children,2)
  field = geom_field.Children(j)
  if (strcmp (field.Name , 'length' ) )
    length = field.Children.Data
  elseif (strcmp (field.Name , 'radius' ) )
    radius = field.Children.Data
  else
    % else freespace
    %keyboard
  end
end
col_size = [radius ',' length];