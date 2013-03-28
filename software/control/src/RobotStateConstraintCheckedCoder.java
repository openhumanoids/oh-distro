import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class RobotStateConstraintCheckedCoder extends RobotStateCoder
{
    drc.robot_state_constraint_checked_t msg_constraint_checked;

    public RobotStateConstraintCheckedCoder (String robot_name, String[] joint_name)
    {
      super(robot_name, joint_name);
      msg_constraint_checked = new drc.robot_state_constraint_checked_t();
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      msg_constraint_checked.robot_state = ((drc.robot_state_t) super.encode(d));

      return msg_constraint_checked;
    }
    
    public LCMEncodable encode(drake.util.CoordinateFrameData d, byte[] constraints_satisfied)
    {
      msg_constraint_checked = ((drc.robot_state_constraint_checked_t) encode(d));
      msg_constraint_checked.num_constraints = constraints_satisfied.length;
      msg_constraint_checked.constraints_satisfied = constraints_satisfied;
      
      return msg_constraint_checked;
    }
}
