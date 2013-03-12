import java.io.*;
import lcm.lcm.*;

public class ActionSequenceListener implements LCMSubscriber
{
	String m_robot_name;
	String m_channel_name;
	boolean m_has_new_message = false;
	ActionSequenceData fdata;

	public ActionSequenceListener(String robot_name,String channel)
	{
		m_robot_name = robot_name;
		m_channel_name = channel;

		LCM lcm = LCM.getSingleton();
		lcm.subscribe(channel,this);
	}

	public synchronized void messageReceived(LCM lcm,String channel,LCMDataInputStream dins)
	{
		try{
			drc.action_sequence_t msg = new drc.action_sequence_t(dins);
			if(msg.robot_name.equals(m_robot_name)){
				fdata = new ActionSequenceData();
				fdata.num_contact_goals = msg.num_contact_goals;
				fdata.object_1_name = new String[msg.num_contact_goals];
				fdata.object_1_grp = new String[msg.num_contact_goals];
				fdata.object_2_name = new String[msg.num_contact_goals];
				fdata.object_2_grp = new String[msg.num_contact_goals];
				fdata.lb_completion_time = new double[msg.num_contact_goals];
				fdata.ub_completion_time = new double[msg.num_contact_goals];
				fdata.contact_types = new int[msg.num_contact_goals];
				fdata.target_pts = new double[3][msg.num_contact_goals];
				fdata.target_pts_relation = new int[3][msg.num_contact_goals];
				fdata.target_pts_radius = new double[msg.num_contact_goals];
				for(int i=0;i<msg.num_contact_goals;i++)
				{
					fdata.object_1_name[i] = new String(msg.contact_goals[i].object_1_name);
					fdata.object_1_grp[i] = new String(msg.contact_goals[i].object_1_contact_grp);
					fdata.object_2_name[i] = new String(msg.contact_goals[i].object_2_name);
					fdata.object_2_grp[i] = new String(msg.contact_goals[i].object_2_contact_grp);
					fdata.lb_completion_time[i] = msg.contact_goals[i].lower_bound_completion_time;
					fdata.ub_completion_time[i] = msg.contact_goals[i].upper_bound_completion_time;
					fdata.contact_types[i] = msg.contact_goals[i].contact_type;
					fdata.target_pts[0][i] = msg.contact_goals[i].target_pt.x;
					fdata.target_pts[1][i] = msg.contact_goals[i].target_pt.y;
					fdata.target_pts[2][i] = msg.contact_goals[i].target_pt.z;
					fdata.target_pts_relation[0][i] = msg.contact_goals[i].x_relation;
					fdata.target_pts_relation[1][i] = msg.contact_goals[i].y_relation;
					fdata.target_pts_relation[2][i] = msg.contact_goals[i].z_relation;
					fdata.target_pts_radius[i] = msg.contact_goals[i].target_pt_radius;
				}
				m_has_new_message = true;
			}
		}
		catch (IOException ex) {
			System.out.println("Exception: "+ex);
		}
	}

	public synchronized ActionSequenceData getNextMessage(long timeout_ms)
	{
		if (m_has_new_message){
			m_has_new_message = false;
			return fdata;
		}

		if(timeout_ms==0)
			return null;
		try{
			if(timeout_ms>0)
				wait(timeout_ms);
			else
				wait();
			if(m_has_new_message){
				m_has_new_message = false;
				return fdata;
			}
		} catch (InterruptedException xcp) {}
		return null;
	}

	public synchronized ActionSequenceData getNextMessage()
	{
		return getNextMessage(-1);
	}

	public synchronized ActionSequenceData getState()
	{
		m_has_new_message = false;
		return fdata;
	}
}
