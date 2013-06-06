import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class ContactStateCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    String m_contact_name;  // todo: make this String[] contact_names

    public ContactStateCoder(String robot_name, String contact_name)
    {
      m_robot_name = robot_name;
      m_contact_name = contact_name;
    }
    
    public int dim()
    {
      return 6;
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.robot_state_t msg = new drc.robot_state_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public drake.util.CoordinateFrameData decode(drc.robot_state_t msg)
    {
      if (msg.robot_name.equals(m_robot_name)) {
        Integer j;
        int index;

        drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
        fdata.val = new double[6];
        fdata.t = (double)msg.utime / 1000000.0;

        boolean b_found = false;
        for (int i=0; i<msg.contacts.num_contacts; i++) {
          if (m_contact_name.equals(msg.contacts.id[i])) {
            fdata.val[0] = msg.contacts.contact_force[i].x;
            fdata.val[1] = msg.contacts.contact_force[i].y;
            fdata.val[2] = msg.contacts.contact_force[i].z;
            fdata.val[3] = msg.contacts.contact_torque[i].x;
            fdata.val[4] = msg.contacts.contact_torque[i].y;
            fdata.val[5] = msg.contacts.contact_torque[i].z;
            b_found = true;
            break;
          }
        }
        if (!b_found) {
          System.out.println("ContactStateCoder: message didn't have a "+m_contact_name+" field");
        }          

        return fdata;
      }
      return null;
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
//      System.out.println("ContactStateCoder: Encode not implemented yet");
      return new drc.robot_state_t();
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
