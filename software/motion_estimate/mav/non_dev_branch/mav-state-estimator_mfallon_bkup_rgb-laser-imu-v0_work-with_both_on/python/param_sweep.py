#!/usr/bin/python
import os
import sys
import numpy
import path_util
import subprocess
import random

test_name = 'particle_mode_sweep_new2'
outdir = path_util.getDataPath() + "/" + test_name
command = path_util.getBasePath() + "/bin/" + "mav-state-estimator" 
#mapFile = "/home/abachrac/data/fixie/fixie-walker-9-15-11/walker-map-9-15.bt_blurred"
#logFile = "/home/abachrac/data/fixie/fixie-walker-9-15-11/quadbase/lcmlog-2011-09-15.08"
#mapFile = path_util.getDataPath() + "/" + "octomap.bt_blurred"
logFile = path_util.getDataPath() + "/" + "icra2012run_vicon/lcmlog_with_init"

paramFile = path_util.getConfigPath() + "/" + "fixie.cfg"
default_args = " -L%s -P%s -R " % (logFile, paramFile)

if not os.path.exists(outdir):
        os.makedirs(outdir)


log2mat = path_util.getBasePath() + "/bin/" + "bot-log2mat" 

sweep_arg1 = "state_estimator.laser.gpf_num_samples"
particle_vals = [25, 35, 46, 66, 87, 124, 164, 233, 308, 436, 577, 818, 1082, 1533, 2027, 2874, 3799, 5386, 7120, 10092]
#particle_vals = map(int, numpy.logspace(numpy.log10(25), numpy.log10(5000), 20))
print particle_vals
param1_vals = map(str, particle_vals)

sweep_arg2 = "state_estimator.laser.gpf_substate"
param2_vals = ['pos_only', 'pos_yaw', 'pos_chi', 'all_states']
#param2_vals = ['all_states']

#sweep_arg2 = "-d"
#param2_vals = map(str, [1, 2, 3, 5, 10, 25, 50, 100])

for it in xrange(100):
    for p1 in param1_vals:
        for p2 in param2_vals:
            nonce = random.randint(0, 1e15)
            print "Running param %s=%s %s=%s" % (sweep_arg1, p1, sweep_arg2, p2)
            out_full_path = "%s/%s_%s_%s_%015d" % (outdir, test_name, p1, p2, nonce)
            lcmurl = "file://%s?mode=w" % (out_full_path)
            lcmurl_command = "LCM_DEFAULT_URL=%s %s" % (lcmurl, command)
            full_args = "%s -O %s=%s|%s=%s|state_estimator.publish_filter_state=false" % (default_args, sweep_arg1, p1, sweep_arg2, p2);
            
            full_command = lcmurl_command + " " + full_args
            print full_command
            #exit(1)
            subprocess.call([command] + full_args.split(), env={"LCM_DEFAULT_URL": lcmurl})#, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            #os.system(full_command)
            
            log2mat_command = "%s %s" % (log2mat, out_full_path)
            print log2mat_command
            subprocess.call([log2mat, out_full_path]) #, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#           os.system(log2mat_command)
            
#            subprocess.call(["gzip", out_full_path]) #, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            print "#######################################################################################################"
            print "#######################################################################################################"
            print "\n\n\n"
        
