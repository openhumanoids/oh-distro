import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class ContactStateCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    String[] m_contact_names;  

    public ContactStateCoder(String robot_name, String[] contact_names)
    {
      m_robot_name = robot_name;
      m_contact_names = new String[contact_names.length];
      System.arraycopy(contact_names,0,m_contact_names,0,contact_names.length);
    }
    
    public int dim()
    {
      return 6*m_contact_names.length;
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
        int index;

        drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
        fdata.val = new double[dim()];
        fdata.t = (double)msg.utime / 1000000.0;

        for (int i=0; i<m_contact_names.length; i++) {
          for (int j=0; j<msg.contacts.num_contacts; j++) {
            if (m_contact_names[i].equals(msg.contacts.id[j])) {
              fdata.val[0+6*i] = msg.contacts.contact_force[j].x;
              fdata.val[1+6*i] = msg.contacts.contact_force[j].y;
              fdata.val[2+6*i] = msg.contacts.contact_force[j].z;
              fdata.val[3+6*i] = msg.contacts.contact_torque[j].x;
              fdata.val[4+6*i] = msg.contacts.contact_torque[j].y;
              fdata.val[5+6*i] = msg.contacts.contact_torque[j].z;
              break;
            }
          }
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
