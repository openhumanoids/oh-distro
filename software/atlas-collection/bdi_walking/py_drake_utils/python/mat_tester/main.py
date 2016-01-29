import scipy.io
import numpy as np

def run_mat_test(in_file, out_file):
    spec = scipy.io.loadmat(in_file)
    python_cmd = str(spec['python_cmd'][0])
    pkg = __import__(str(spec['python_pkg'][0]), globals(), locals(), python_cmd)
    args = np.reshape(spec['args'], [-1])
    result = pkg.__dict__[python_cmd](args)
    # result = np.reshape(result, [-1])
    result = np.array(result)
    scipy.io.savemat(out_file, {'result': result},oned_as='column')

def main():
    import sys
    in_file, out_file = sys.argv[1:3]
    run_mat_test(in_file, out_file)

if __name__ == '__main__':
    main()
