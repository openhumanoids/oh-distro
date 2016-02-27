package drc.control;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;

public class ForceTorqueStateCoder implements drake.util.LCMCoder
{
    public ForceTorqueStateCoder() {}
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        bot_core.robot_state_t msg = new bot_core.robot_state_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public int dim() {
      return 18;
    }

    public drake.util.CoordinateFrameData decode(bot_core.robot_state_t msg)
    {
      int index;
      int dim=18; // 3 per foot, 6 per hand
      drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
      fdata.val = new double[18];
      fdata.t = (double)msg.utime / 1000000.0;

      fdata.val[0] = msg.force_torque.l_foot_force_z;
      fdata.val[1] = msg.force_torque.l_foot_torque_x;
      fdata.val[2] = msg.force_torque.l_foot_torque_y;

      fdata.val[3] = msg.force_torque.r_foot_force_z;
      fdata.val[4] = msg.force_torque.r_foot_torque_x;
      fdata.val[5] = msg.force_torque.r_foot_torque_y;

      fdata.val[6] = msg.force_torque.l_hand_force[0];
      fdata.val[7] = msg.force_torque.l_hand_force[1];
      fdata.val[8] = msg.force_torque.l_hand_force[2];
      fdata.val[9] = msg.force_torque.l_hand_torque[0];
      fdata.val[10] = msg.force_torque.l_hand_torque[1];
      fdata.val[11] = msg.force_torque.l_hand_torque[2];

      fdata.val[12] = msg.force_torque.r_hand_force[0];
      fdata.val[13] = msg.force_torque.r_hand_force[1];
      fdata.val[14] = msg.force_torque.r_hand_force[2];
      fdata.val[15] = msg.force_torque.r_hand_torque[0];
      fdata.val[16] = msg.force_torque.r_hand_torque[1];
      fdata.val[17] = msg.force_torque.r_hand_torque[2];

      return fdata;
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
//      System.out.println("ContactStateCoder: Encode not implemented yet");
      return new bot_core.robot_state_t();
    }
    
    public String timestampName()
    {
      return "utime";
    }
		
		public String[] coordinateNames() {
			String[] coords = new String[dim()];
			Arrays.fill(coords, "");
			return coords;
		}
}
