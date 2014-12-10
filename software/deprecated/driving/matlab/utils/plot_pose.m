function plot_pose(R,T,label,scale)

R = R';

held = ishold;
hold on;
plot3(T(1),T(2),T(3),'k.','MarkerSize',10);
plot3([T(1);T(1)+scale*R(1,1)],[T(2);T(2)+scale*R(1,2)],[T(3);T(3)+scale*R(1,3)],'r-');
plot3([T(1);T(1)+scale*R(2,1)],[T(2);T(2)+scale*R(2,2)],[T(3);T(3)+scale*R(2,3)],'g-');
plot3([T(1);T(1)+scale*R(3,1)],[T(2);T(2)+scale*R(3,2)],[T(3);T(3)+scale*R(3,3)],'b-');
if (numel(label) > 0)
    text(T(1),T(2),T(3),label);
end
hold off;

if (held)
    hold on;
end
