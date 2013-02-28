import java.io.*;
import lcm.lcm.*;

public class GraspOptStatusPublisher
{
    drc.grasp_opt_status_t msg;
    String channel_name;
    //const int16_t SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5;
    public GraspOptStatusPublisher(short num_matlab_workers, String channel)
    {
	    msg = new drc.grasp_opt_status_t();
	    msg.num_matlab_workers = num_matlab_workers;
	    msg.matlab_pool_ready=  false; 
	    msg.worker_id = 1;
	    msg.worker_available = false;
	    channel_name = channel;
    }

    public void publish(long utime, short worker_id, boolean worker_available) // use doubles here for compatibility w/ matlab
    {
	    LCM lcm = LCM.getSingleton();
	    msg.utime = utime; //System.nanoTime()/1000; // THIS SHOULD NOT BE SYSTEM TIME,  use the INIT_GRASP_SEED time
	    msg.matlab_pool_ready = true;
	    msg.worker_id = worker_id;
	    msg.worker_available = worker_available;	    
	    lcm.publish(channel_name,msg);
    }   

}
