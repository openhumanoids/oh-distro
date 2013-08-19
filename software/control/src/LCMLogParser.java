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
import java.util.*;

/**
 *  Class for parsing an LCM log into arrays (designed to be used with MATLAB, which is why ArrayList is not used)
 *  General use scenario:
 *  LCMLogParser parser = new LCMLogParser();
 *  parser.addChannel(String lcm_channel_name, LCMCoder coder);  //Register a channel name to parse and associated coder. Can add more than one
 *  parser.parseLog(String log_file_name);
 *  parser.getT(String lcm_channel_name); //Get time series from parsed log
 *  parser.getData(String lcm_channel_name); //Get data series from parsed log
 */
public class LCMLogParser {
  private static final int INIT_LOG_SIZE = 1000;

  private class ChannelLog {
    public final String m_channel_name;
    public final LCMCoder m_coder;
    private int m_message_size = -1;
    protected double[] m_t;
    protected double[] m_data;
    private int m_data_length = 0;

    public ChannelLog(String channel, LCMCoder coder) {
      m_channel_name = channel;
      m_coder = coder;
      m_t = new double[INIT_LOG_SIZE];
    }

    /**
     * Ensure that the log has size to add a new item. If it does not, double the size
     */
    public void ensureSize() {
      if (m_data_length == m_t.length) {
        setLength(2*m_t.length);
      }
    }

    private void setLength(int length) {
      if (m_message_size == -1) {
        return; //do nothing
      }
      int copy_length = length > m_t.length ? m_t.length : length;
//      System.out.println("Resizing data from " + m_t.length + " to " + length);
      double[] tmp = new double[length*m_message_size];
      System.arraycopy(m_data, 0, tmp, 0, copy_length*m_message_size);
      m_data = tmp;
      
      tmp = new double[length];
      System.arraycopy(m_t, 0, tmp, 0, copy_length);
      m_t = tmp;

    }
    /**
      * Shrink the array sizes to the data length
      */
    public void resizeToData() {
      setLength(m_data_length);
    }

    public void addEntry(byte[] byte_data) {
      CoordinateFrameData data = m_coder.decode(byte_data);
      if (m_message_size == -1) {
        //Do initialization
        m_message_size = data.val.length;
        m_data = new double[INIT_LOG_SIZE*m_message_size];
      } else {
        if (m_message_size != data.val.length) {
          throw new IllegalStateException("New state message on channel " + m_channel_name + " has length " + data.val.length + ", but expected " + m_message_size);
        }
      }
      ensureSize();

      System.arraycopy(data.val, 0, m_data, m_data_length*m_message_size, m_message_size);
      m_t[m_data_length] = data.t;
      m_data_length++;
    }
  }


  private final Map<String,ChannelLog> m_channel_map = new HashMap<String,ChannelLog>();


/**
* public void parseLog(String filename)
*   Parse a log file to memory, returns 0 on success, 1 if it failed to load the file.
*/
  public int parseLog(String filename) {
    try {
      Log log = new Log(filename,"r");
      try {
        while(true) {
          Log.Event e = log.readNext();
  //        System.out.println(e.channel);
          ChannelLog c = m_channel_map.get(e.channel);
          if (c != null) {
            c.addEntry(e.data);
          }
        }
      } catch (EOFException ex) {
  //      System.out.println(ex);
      } finally {
        log.close();
      }
    } catch (IOException ex) {
      System.err.println(ex);
      return 1;
    }

    for (ChannelLog c : m_channel_map.values()) {
      c.resizeToData();
    }
    return 0;
  }

  /**
  * Get the time series for data from a given channel, after parsing a log
  */
  public double[] getT(String channel) {
    ChannelLog c = m_channel_map.get(channel);
    if (c == null) {
      return null;
    }

    return c.m_t;
  }


  /**
  * Get the the data, after parsing a log.  Returned as a single stacked vector [x(t0);x(t1);...;x(tN)]
  */
  public double[] getData(String channel) {
    ChannelLog c = m_channel_map.get(channel);
    if (c == null) {
      return null;
    }

    return c.m_data;
  }

  /**
  * Add a channel to parse
  */ 
  public void addChannel(String channel, LCMCoder coder) {
    m_channel_map.put(channel, new ChannelLog(channel,coder));
  }

  /**
  * Remove channel from list to parse
  */
  public void removeChannel(String channel) {
    m_channel_map.remove(channel);
  }
}
