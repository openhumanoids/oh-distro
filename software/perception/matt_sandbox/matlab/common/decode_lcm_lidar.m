function scan = decode_lcm_lidar(in)

scan.ranges = double(in.ranges);
scan.intensities = double(in.intensities);
scan.theta_min = in.rad0;
scan.theta_step = in.radstep;
scan.utime = int64(in.utime);