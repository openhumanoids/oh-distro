package drc.control;

import drake.util.BotLCMGLClient;

public class LCMGLLineBuffer {
  private BotLCMGLClient m_client;
  public static final int BUFFER_SIZE = 1000;
  private float m_r;
  private float m_g;
  private float m_b;
  private double[][] m_points = new double[BUFFER_SIZE][3];
  private int m_index = 0;
  private int m_length = 0;

  public LCMGLLineBuffer(BotLCMGLClient lcmClient, double red, double green, double blue) {
    m_client = lcmClient;
    m_r = (float) red;
    m_g = (float) green;
    m_b = (float) blue;
  }

  public void addPointAndDisplay(double x, double y, double z) {
    m_points[m_index][0] = x;
    m_points[m_index][1] = y;
    m_points[m_index][2] = z;

    if (m_length < BUFFER_SIZE)
      m_length++;
    
    if (m_index == BUFFER_SIZE - 1)
      m_index = 0;
    else
      m_index++;
    
    m_client.glColor3f(m_r,m_g,m_b);
    m_client.glBegin(m_client.LCMGL_LINES);

    for (int i=0; i < m_length; i++) {
      int j = m_index - i - 1;
      if (j < 0) 
        j = j + BUFFER_SIZE;
      
      if (i != 0)
        m_client.glVertex3d(m_points[j][0], m_points[j][1], m_points[j][2]);

      m_client.glVertex3d(m_points[j][0], m_points[j][1], m_points[j][2]);
    }
    m_client.glEnd();
    m_client.switchBuffers();
  }
}
