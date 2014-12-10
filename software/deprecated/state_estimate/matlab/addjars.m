% addpath 'generic_functions'
% addpath 'generic_functions/kf'

% javaaddpath '../../../build/share/java/lcmtypes_drc_lcmtypes.jar'
javaaddpath([getenv('DRC_BASE'),'/software/build/share/java/lcmtypes_drc_lcmtypes.jar'])

% lcm_java_classpath = getCMakeParam('lcm_java_classpath');
%   if ~isempty(lcm_java_classpath)
%     javaaddpath(lcm_java_classpath);
%     disp(' Added the lcm jar to your javaclasspath (found via cmake)');
%     conf.lcm_enabled = logical(exist('lcm.lcm.LCM','class'));
%   end

[retval,cp] = system('pkg-config --variable=classpath lcm-java');
  if (retval==0 && ~isempty(cp))
    disp(' Added the lcm jar to your javaclasspath (found via pkg-config)');
    javaaddpath(strtrim(cp));
  end
