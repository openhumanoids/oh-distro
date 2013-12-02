function lcmglDrawHT(lcmgl,T)
  aa = rotmat2axis(T(1:3,1:3));
  lcmgl.glTranslated(T(1,4),T(2,4),T(3,4));
  lcmgl.glRotated(aa(4)*180/pi,aa(1),aa(2),aa(3));
  lcmgl.glDrawAxes();
  lcmgl.glRotated(-aa(4)*180/pi,aa(1),aa(2),aa(3));
  lcmgl.glTranslated(-T(1,4),-T(2,4),-T(3,4));
end