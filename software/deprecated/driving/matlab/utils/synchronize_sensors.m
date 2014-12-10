function sensor_inds = synchronize_sensors(in,t_out)

t_out = t_out(:);
sensor_inds = closest_indices([in.t], t_out);
out_inds = find(sensor_inds>0);
sensor_inds = sensor_inds(out_inds);
