function write_config(pose,filename)

fp = fopen(filename,'w');
fprintf(fp, '\n');

sp = '    ';
for i = 1:numel(pose)
    T = pose(i).T;
    rpy = quat2rpy(rot2quat(pose(i).R))*180/pi;
    fprintf(fp, '%s%s%s {\n', sp,sp,pose(i).name);
    if (isfield(pose(i),'extra') && numel(pose(i).extra)>0)
        fprintf(fp, '%s%s%swidth = %d;\n', sp,sp,sp,pose(i).extra.radial_params.width);
        fprintf(fp, '%s%s%sheight = %d;\n', sp,sp,sp,pose(i).extra.radial_params.height);
        fprintf(fp, '%s%s%sdistortion_model = "spherical";\n',sp,sp,sp);
        fprintf(fp, '%s%s%sdistortion_center = [%f, %f];\n', sp,sp,sp,pose(i).extra.radial_params.center);
        fprintf(fp, '%s%s%sdistortion_params = [%f];\n', sp,sp,sp,pose(i).extra.radial_params.a);
        K = pose(i).extra.K;
        fprintf(fp, '%s%s%spinhole = [%f, %f, %f, %f, %f];\n', sp,sp,sp,K(1,1), K(2,2), K(1,2), K(1,3), K(2,3));
    end
    fprintf(fp, '%s%s%sposition = [%f, %f, %f];\n', sp,sp,sp,T);
    fprintf(fp, '%s%s%srpy = [%f, %f, %f];\n', sp,sp,sp,rpy);
    if (isfield(pose(i), 'relative_to'))
        fprintf(fp, '%s%s%srelative_to = "%s";\n',sp,sp,sp,pose(i).relative_to);
    else
        fprintf(fp, '%s%s%srelative_to = "body";\n',sp,sp,sp);
    end
    fprintf(fp, '%s%s}\n\n',sp,sp);
end

fclose(fp);
