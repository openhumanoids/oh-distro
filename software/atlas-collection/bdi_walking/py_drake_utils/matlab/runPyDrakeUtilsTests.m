try
  to_test = {{'quat2rpy', 'py_drake_utils', 'quat2rpy', rand(4, 10) * 2 - 1},...
             {'rpy2rotmat', 'py_drake_utils', 'rpy2rotmat', rand(3, 10) * 2*pi - pi},...
            };

  for j = 1:length(to_test)
    matlab_cmd = to_test{j}{1}
    python_pkg = to_test{j}{2};
    python_cmd = to_test{j}{3};
    for k = 1:size(to_test{j}{4}, 2)
      args = to_test{j}{4}(:,k);
      save('tmp_in', 'python_cmd', 'python_pkg', 'args');
      system('drc-py-mat-tester tmp_in.mat tmp_out.mat');
      out_data = load('tmp_out');
      py_result = out_data.result;
      mat_result = feval(matlab_cmd, args);
      valuecheck(py_result, mat_result);
    end
  end

  delete('tmp_in.mat')
  delete('tmp_out.mat')
catch
  exit(1);
end
