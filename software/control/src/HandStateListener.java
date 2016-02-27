package drc.control;

import java.io.*;
import lcm.lcm.*;
import java.util.*;

public class HandStateListener implements LCMSubscriber
{
  java.util.TreeMap<String,Integer> m_joint_map;
  String[] m_joint_names;
  int hand_dim;
  boolean setMapFlag;
  double[] hand_state;
  boolean m_has_new_message = false;
  

  public HandStateListener(int hand_mode, String prefix, String channel)
  {
    LCM lcm = LCM.getSingleton();
    lcm.subscribe(channel,this);

    if(hand_mode == 0)
    {
      hand_dim = 0;
      System.out.println("No hand listener for "+prefix+" hand");
      setMapFlag = true;
    }
    else if(hand_mode == 1)
    {
      hand_dim = 12;
      m_joint_names = new String[hand_dim];
      for(int i = 0;i<4;i++)
      {
        for(int j = 0;j<3;j++)
        {
          int ind = i*3+j;
          m_joint_names[ind]= String.format("%s_f%d_j%d",prefix,i,j);
        }
      }
      System.out.println("construct sandia hand listener for "+prefix+" hand");
      setMapFlag = false;
    }
    else if(hand_mode == 2)
    {
      hand_dim = 0;
      System.out.println("construct irobot hand listener for "+prefix+" hand");
      setMapFlag = true;
      // This needs to be changed when irobot hand can be parsed
    }
  }

  public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
  {
    try
    {
      bot_core.robot_state_t msg = new bot_core.robot_state_t(dins);
      if(!setMapFlag)
      {
        java.util.TreeMap<String,Integer> msg_joint_map = new java.util.TreeMap<String,Integer>();
        for(int i = 0;i<msg.num_joints;i++)
        {
          Integer joint_idx = new Integer(i);
          msg_joint_map.put(msg.joint_name[i],joint_idx);
          System.out.format("%s %d\n",msg.joint_name[i],i);
        }
        m_joint_map = new java.util.TreeMap<String,Integer>();
        for(int i = 0;i<hand_dim;i++)
        {
          System.out.format("%s\n",m_joint_names[i]);
          if(msg_joint_map.get(m_joint_names[i])==null)
          {
            System.out.format("%s is not in the map during construction\n",m_joint_names[i]);
          }
          m_joint_map.put(m_joint_names[i],msg_joint_map.get(m_joint_names[i]));
        }
        setMapFlag = true;
      }
      hand_state = new double[hand_dim*2];
      for(int i = 0;i<hand_dim;i++)
      {
        if(m_joint_map.get(m_joint_names[i])== null)
        {
          System.out.format("%s is not in the map\n",m_joint_names[i]);
        }
        int hand_joint_idx = m_joint_map.get(m_joint_names[i]);
        hand_state[i] = msg.joint_position[hand_joint_idx];
        hand_state[i+hand_dim] = msg.joint_velocity[hand_joint_idx];
      }
      m_has_new_message = true;
    }
    catch (IOException ex)
    {
      System.out.println("Exception: "+ex);
    }
  }
    public synchronized double[] getNextMessage(long timeout_ms)
    {
      if (m_has_new_message) {
        m_has_new_message = false;
        return hand_state;
      }

      if(timeout_ms == 0)
        return null;

      try {
        if(timeout_ms > 0)
          wait(timeout_ms);
        else
          wait();
        
        if (m_has_new_message) {
          m_has_new_message = false;
          return hand_state;
        }
      } catch (InterruptedException xcp) { }
      
      return null;
    }

    public synchronized double[] getNextMessage()
    {
      return getNextMessage(-1);
    }
    
    public synchronized double[] getState()
    {
      m_has_new_message = false;
      return hand_state;
    }

}
