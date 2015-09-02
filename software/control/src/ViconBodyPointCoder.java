package drc.control;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;
import vicon.*;

public class ViconBodyPointCoder implements drake.util.LCMCoder 
{
  private int m_dim = -1;
  private int m_nummodels = -1;
  private int[] m_modelDim;
  private String[] m_modelNames;
  public ViconBodyPointCoder() {
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

    if (m_dim == -1) {
      m_dim = message_size;
    } else {
      if (m_dim != message_size) {
        throw new IllegalStateException("Message changed in length, expected " + m_dim + " and got " + message_size);
      }
    }

    if (m_nummodels == -1) {
      m_nummodels = msg.nummodels;
      m_modelDim = new int[m_nummodels];
      m_modelNames = new String[m_nummodels];
      for (int i=0; i<m_nummodels; i++) {
        m_modelDim[i] = msg.models[i].nummarkers;
        m_modelNames[i] = msg.models[i].name;
      }
    } else {
      if (m_nummodels != msg.nummodels) {
        throw new IllegalStateException("Number of models changed, expected " + m_nummodels + " and got " + msg.nummodels);
      }
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

  public int getNumModels() {
    return m_nummodels;
  }

  public int[] getModelDim() {
    return m_modelDim;
  }
  public String getModelName(int index) {
    return m_modelNames[index];
  }
	
		public String[] coordinateNames() {
			String[] coords = new String[dim()];
			Arrays.fill(coords, "");
			return coords;
		}
}
