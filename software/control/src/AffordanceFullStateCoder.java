import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class AffordanceFullStateCoder implements drake.util.LCMCoder
{
    String m_otdf_type;
    int m_uid;
    int m_num_floating_joints;
    int m_num_states;
    java.util.TreeMap<String,Integer> m_state_map;
    drc.affordance_collection_t msg;
    
    public AffordanceFullStateCoder(String otdf_type,Integer uid, String[] state_names)
    {
      msg = new drc.affordance_collection_t();
      m_otdf_type = otdf_type;
      if(uid != null) {
        m_uid = uid;
      } else {
        m_uid = -1;
      }
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
        drc.affordance_collection_t msg = new drc.affordance_collection_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public drake.util.CoordinateFrameData decode(drc.affordance_collection_t msg)
    {
      for (int i=0;i<msg.naffs;i++) {
        if (msg.affs[i].otdf_type.equals(m_otdf_type) && (m_uid == -1 || m_uid == msg.affs[i].uid)) {
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();

          fdata.val = new double[(m_num_floating_joints+m_num_states)];
          fdata.t = (double)msg.utime / 1000000.0;

          fdata.val[0] = msg.affs[i].origin_xyz[0];
          fdata.val[1] = msg.affs[i].origin_xyz[1];
          fdata.val[2] = msg.affs[i].origin_xyz[2];
          fdata.val[3] = msg.affs[i].origin_rpy[0];
          fdata.val[4] = msg.affs[i].origin_rpy[1];
          fdata.val[5] = msg.affs[i].origin_rpy[2];

          Integer k;
          int index;
          for (int j=0; j<m_num_states; j++) {
            k = m_state_map.get(msg.affs[j].state_names[j]);
            if (k!=null) {
              index = k.intValue();
              fdata.val[index+m_num_floating_joints] = msg.affs[j].states[j];
              fdata.val[index+m_num_states+2*m_num_floating_joints] = 0.0;
            }
          }
          return fdata;
        }
      }
      return null;
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      // don't need to go in this direction, except to determine message
      // monitor types, so return empty affordance_collection_t message.
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
