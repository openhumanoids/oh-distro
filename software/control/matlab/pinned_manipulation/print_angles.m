function print_angles(T,tag)

angles = rotmat2rpy(T(1:3,1:3))*(180/pi);
if(nargin==2)
   disp([tag ': ' num2str(angles(1)) ' ' num2str(angles(2)) ' ' num2str(angles(3))]); 
   return;
end

disp([num2str(angles(1)) ' ' num2str(angles(2)) ' ' num2str(angles(3))]);
end