package drc.control;

import java.io.*;
import java.lang.*;
import lcm.lcm.*;
import lcm.logging.Log;
import java.util.ArrayList;
import java.lang.String;
import java.lang.System;
import drake.util.LCMCoder;
import drake.util.CoordinateFrameData;

public class AtlasStateCommandLogParser {
  private static final int INIT_LOG_SIZE = 1000;
  private final int m_state_hash;
  private final int m_input_hash;
  private final LCMCoder m_state_coder;
  private final LCMCoder m_input_coder;
  private final int m_state_size;
  private final int m_input_size;

  private double[] m_t_u;
  private double[] m_t_x;
  private double[] m_x_data;
  private double[] m_u_data;

  public AtlasStateCommandLogParser(int state_size, int input_size, String state_channel, LCMCoder state_coder, String input_channel, LCMCoder input_coder) {
    m_state_hash = state_channel.hashCode();
    m_input_hash = input_channel.hashCode();
    m_state_coder = state_coder;
    m_input_coder = input_coder;

    m_t_u = new double[INIT_LOG_SIZE];
    m_t_x = new double[INIT_LOG_SIZE];
    m_x_data = new double[INIT_LOG_SIZE*state_size];
    m_u_data = new double[INIT_LOG_SIZE*input_size];
    m_state_size = state_size;
    m_input_size = input_size;
  }

/**
* public void parseLog(String filename)
*   Parse a log file to memory, returns 0 on success, 1 if it failed to load the file.
*/
  public int parseLog(String filename) {
    int state_ind = 0;
    int input_ind = 0;
    try {
      Log log = new Log(filename,"r");
      while(true) {
        Log.Event e = log.readNext();
//        System.out.println(e.channel);
        int msg_hash = e.channel.hashCode();
        if (msg_hash == m_state_hash) {
          //Resize arrays if necessary
          if (state_ind == m_t_x.length) {
            double[] tmp = new double[2*m_t_x.length*m_state_size];
            System.arraycopy(m_x_data, 0, tmp, 0, m_t_x.length*m_state_size);
            m_x_data = tmp;
            
            tmp = new double[2*m_t_x.length];
            System.arraycopy(m_t_x, 0, tmp, 0, m_t_x.length);
            m_t_x = tmp;
//            System.out.println("Resizing state");
          }

          CoordinateFrameData data = m_state_coder.decode(e.data);
          System.arraycopy(data.val, 0, m_x_data, state_ind*m_state_size, m_state_size);
          m_t_x[state_ind] = data.t;
          state_ind++;
        } else if (msg_hash == m_input_hash) {
          //Resize arrays if necessary
          if (input_ind == m_t_u.length) {
            double[] tmp = new double[2*m_t_u.length*m_input_size];
            System.arraycopy(m_u_data, 0, tmp, 0, m_t_u.length*m_input_size);
            m_u_data = tmp;
            
            tmp = new double[2*m_t_u.length];
            System.arraycopy(m_t_u, 0, tmp, 0, m_t_u.length);
            m_t_u = tmp;
//            System.out.println("Resizing input");
          }

          CoordinateFrameData data = m_input_coder.decode(e.data);
          System.arraycopy(data.val, 0, m_u_data, input_ind*m_input_size, m_input_size);
          m_t_u[input_ind] = data.t;
          input_ind++;
        }
      }
    } catch (EOFException ex) {
//      System.out.println(ex);
    } catch (IOException ex) {
      System.out.println(ex);
      return 1;
    }

    //Clean-up sizes
    double[] tmp = new double[state_ind*m_state_size];
    System.arraycopy(m_x_data, 0, tmp, 0, state_ind*m_state_size);
    m_x_data = tmp;

    tmp = new double[state_ind];
    System.arraycopy(m_t_x, 0, tmp, 0, state_ind);
    m_t_x = tmp;

    tmp = new double[input_ind*m_input_size];
    System.arraycopy(m_u_data, 0, tmp, 0, input_ind*m_input_size);
    m_u_data = tmp;

    tmp = new double[input_ind];
    System.arraycopy(m_t_u, 0, tmp, 0, input_ind);
    m_t_u = tmp;
    return 0;
  }

/**
* Get the time series for the state data, after parsing a log
*/
  public double[] getTx() {
    return m_t_x;
  }

/**
* Get the time series for the input data, after parsing a log
*/
  public double[] getTu() {
    return m_t_u;
  }

/**
* Get the the state data, after parsing a log.  Returned as a single stacked vector [x(t0);x(t1);...;x(tN)]
*/
  public double[] getStateData() {
    return m_x_data;
  }

/**
* Get the the input data, after parsing a log.  Returned as a single stacked vector [u(t0);u(t1);...;u(tN)]
*/
  public double[] getInputData() {
    return m_u_data;
  }
}
