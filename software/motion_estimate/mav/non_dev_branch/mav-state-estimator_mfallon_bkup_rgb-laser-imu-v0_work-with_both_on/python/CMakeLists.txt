find_package(PythonInterp REQUIRED)

# executable scripts:  script-name  python-module
pods_install_python_script(mav-state-estimator-param-sweep param_sweep.py)
