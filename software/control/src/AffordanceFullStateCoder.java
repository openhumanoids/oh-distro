import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class AffordanceFullStateCoder implements drake.util.LCMCoder
{
    int m_uid;
    int m_num_floating_joints;
    int m_num_states;
    java.util.TreeMap<String,Integer> m_state_map;
    
    //Old properties---------------------------------------------------------------
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_floating_joint_map;
    drc.robot_state_t msg;
    //-----------------------------------------------------------------------------

    public AffordanceFullStateCoder(int uid, String[] state_names)
    {
      m_uid = uid;
      m_num_floating_joints = 6;
      m_state_map = new java.util.TreeMap<String,Integer>();
      m_num_states = 0;
      if(state_names != null && state_names.length > 0) {
        for (int i=0; i<state_names.length; i++) {
          m_state_map.put(state_names[i],i);
          m_num_states+=1;
        }     
      }
    }
    
    public int dim()
    {
      return 2*(m_num_states+m_num_floating_joints);
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.affordance_t msg = new drc.affordance_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public drake.util.CoordinateFrameData decode(drc.affordance_t msg)
    {
      if (msg.uid == m_uid) {
        drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();

        fdata.val = new double[(m_num_floating_joints+m_num_states)];
        fdata.t = (double)msg.utime / 1000000.0;

        fdata.val[0] = msg.origin_xyz[0];
        fdata.val[1] = msg.origin_xyz[1];
        fdata.val[2] = msg.origin_xyz[2];
        fdata.val[3] = msg.origin_rpy[0];
        fdata.val[4] = msg.origin_rpy[1];
        fdata.val[5] = msg.origin_rpy[2];

        Integer j;
        int index;
        for (int i=0; i<msg.nstates; i++) {
          j = m_state_map.get(msg.state_names[i]);
          if (j!=null) {
            index = j.intValue();
            fdata.val[index+m_num_floating_joints] = msg.states[i];
            fdata.val[index+m_num_states+2*m_num_floating_joints] = 0.0;
          }
        }
        return fdata;
      }
      return null;
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      // don't need to go in this direction 
      return null;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
