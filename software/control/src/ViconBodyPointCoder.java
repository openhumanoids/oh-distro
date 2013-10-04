package drc.control;

import java.io.*;
import java.lang.*;
import lcm.lcm.*;
import viconstructs.*;

public class ViconBodyPointCoder implements drake.util.LCMCoder 
{
  private int m_dim;
  public ViconBodyPointCoder(int dim) {
    m_dim = dim;
  }

  public drake.util.CoordinateFrameData decode(byte[] data) {
    try {
      vicon_t msg = new vicon_t(data);
      return decode(msg);
    } catch (IOException ex) {
      System.out.println("Exception: " + ex);
    }
    return null;
  }
  
  public drake.util.CoordinateFrameData decode(vicon_t msg) { 
    drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
    int message_size = 0;
    for (int i=0; i<msg.nummodels; i++) {
      message_size += msg.models[i].nummarkers*4; //[x,y,z,o] just capturing the markers, for now
    }

    fdata.val = new double[message_size];
    fdata.t = (double)msg.utime / 1000000.0;

    int data_index = 0;
    for (int i=0; i<msg.nummodels; i++) {
      for (int j=0; j<msg.models[i].nummarkers; j++) {
        fdata.val[data_index] = msg.models[i].markers[j].xyz[0];
        fdata.val[data_index+1] = msg.models[i].markers[j].xyz[1];
        fdata.val[data_index+2] = msg.models[i].markers[j].xyz[2];
        fdata.val[data_index+3] = msg.models[i].markers[j].o;
        data_index += 4;
      }
    }
    return fdata;
  }

  public LCMEncodable encode(drake.util.CoordinateFrameData d) {
    return null;
  }

  public String timestampName() {
    return "utime";
  }

  public int dim() {
    return m_dim;
  }
}
