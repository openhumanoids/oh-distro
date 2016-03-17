javaaddpath('/usr/local/share/java/lcm.jar');
jar_dir = [getenv('DRC_BASE'),'/software/build/share/java'];
d = dir([jar_dir,'/lcmtypes_*.jar']);
for i = 1:numel(d)
    javaaddpath([jar_dir,'/',d(i).name]);
end
