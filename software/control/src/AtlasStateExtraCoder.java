package drc.control;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;

public class AtlasStateExtraCoder implements drake.util.LCMCoder
{
    short m_num_joints;
    
    public AtlasStateExtraCoder(short num_joints) {
      m_num_joints = num_joints;
    }
    
    public int dim() {
      return 4*m_num_joints;
    }

    public drake.util.CoordinateFrameData decode(byte[] data) {
      try {
        atlas.state_extra_t msg = new atlas.state_extra_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public drake.util.CoordinateFrameData decode(atlas.state_extra_t msg) {
      Integer j;
      int index;

      drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
      fdata.val = new double[dim()];
      fdata.t = (double)msg.utime / 1000000.0;
      for (int i=0; i<m_num_joints; i++) {
        fdata.val[i] = msg.joint_position_out[i];
        fdata.val[m_num_joints + i] = msg.joint_velocity_out[i];
        fdata.val[2*m_num_joints + i] = msg.psi_pos[i];
        fdata.val[3*m_num_joints + i] = msg.psi_neg[i];
      }
      return fdata;
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d) {
      atlas.state_extra_t msg = new atlas.state_extra_t();
      msg.utime = (long)(d.t*1000000);
      msg.joint_position_out = new float[m_num_joints];
      msg.joint_velocity_out = new float[m_num_joints];
      msg.psi_pos = new float[m_num_joints];
      msg.psi_neg = new float[m_num_joints];
      for (int i=0; i<m_num_joints; i++) {
        msg.joint_position_out[i] = (float) d.val[i];
        msg.joint_velocity_out[i] = (float) d.val[m_num_joints + i];
        msg.psi_pos[i] = (float) d.val[2*m_num_joints + i];
        msg.psi_neg[i] = (float) d.val[3*m_num_joints + i];
      }
      return msg;
    }

    public String timestampName() {
      return "utime";
    }
		
		public String[] coordinateNames() {
			String[] coords = new String[dim()];
			Arrays.fill(coords, "");
			return coords;
		}
}
