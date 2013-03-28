import java.io.*;
import lcm.lcm.*;

public class RobotPlanConstraintCheckedPublisher extends RobotPlanPublisher
{
    drc.robot_plan_constraint_checked_t msg_constraint_checked;

    public RobotPlanConstraintCheckedPublisher(String robot_name,String[] joint_name,boolean floating_base, String channel)
    {
      super(robot_name, joint_name, floating_base, channel);
      msg_constraint_checked = new drc.robot_plan_constraint_checked_t();
    }

    public LCMEncodable encode(double[] t, double[][] x)
    {
      msg_constraint_checked.robot_plan = ((drc.robot_plan_t) super.encode(t,x));

      return msg_constraint_checked;
    }

    public LCMEncodable encode(double[] t, double[][] x, byte[][] constraints_satisfied)
    {
      msg_constraint_checked = ((drc.robot_plan_constraint_checked_t) encode(t,x));
      msg_constraint_checked.num_constraints = constraints_satisfied.length;
      msg_constraint_checked.constraints_satisfied = constraints_satisfied;
      
      return msg_constraint_checked;
    }

    public void publish(double[] t,double[][] x, byte[][] constraints_satisfied)
    {
      if (t.length != msg.num_states) {
        allocate(t.length);  // reallocate on-demand
      }
      if (x[0].length != t.length) {
        // todo: throw an exception here instead
        System.out.format("Error: x[0].length=%d and t.length=%d\n",x[0].length,t.length);
        return;
      }
      if (constraints_satisfied[0].length != t.length) {
        // todo: throw an exception here instead
        System.out.format("Error: constraints_satisfied[0].length=%d and t.length=%d\n",constraints_satisfied[0].length,t.length);
        return;
      }
      LCM lcm = LCM.getSingleton();
      lcm.publish(channel_name,encode(t,x,constraints_satisfied));
    }
}

