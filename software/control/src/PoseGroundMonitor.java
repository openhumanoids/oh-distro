
import java.io.*;
import java.lang.*;
import lcm.lcm.*;

// listens for POSE_GRAPH messages and returns the most recently received height 
// i've special-cased this monitor because the output need to be fast because 
// it's called inside contact constraints.
// (and only needs a small fraction of the data available in the message)

public class PoseGroundMonitor implements LCMSubscriber
{
  double m_z;
 
  public PoseGroundMonitor()
  {
    LCM lcm = LCM.getSingleton();
    lcm.subscribe("POSE_GROUND",this);
  }

  public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
  {
    try {
      byte[] data = new byte[dins.available()];
      dins.readFully(data);
      bot_core.pose_t msg = new bot_core.pose_t(data);
      m_z = msg.pos[2];
    } catch (IOException ex) {
      System.out.println("Exception: " + ex);
    }
  }
  
  public synchronized double getHeight() {
    return m_z;
  }
  
}