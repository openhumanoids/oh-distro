import lcm
import drc as lcmdrc
import drake as lcmdrake

class ControllerContactPassthrough(object):
    def __init__(self, qp_input_channel='QP_CONTROLLER_INPUT',
                 output_channel='CONTROLLER_FOOT_CONTACT',
                 body_ids={'right': 25, 'left': 12}):
        self.qp_input_channel = qp_input_channel
        self.output_channel = output_channel
        self.body_ids = body_ids
        self.lc = lcm.LCM()
        self._setupSubscritions()

    def _setupSubscritions(self):
        self.lc.subscribe(self.qp_input_channel, self.handle_qp_input)

    def handle_qp_input(self, channel, msg):
        if isinstance(msg, str):
            msg = lcmdrake.lcmt_qp_controller_input.decode(msg)

        contact_msg = lcmdrc.controller_foot_contact_t()

        for support_data in msg.support_data:
            if support_data.body_id == self.body_ids['right']:
                contact_msg.num_right_foot_contacts = support_data.num_contact_pts
                contact_msg.right_foot_contacts = support_data.contact_pts
            elif support_data.body_id == self.body_ids['left']:
                contact_msg.num_left_foot_contacts = support_data.num_contact_pts
                contact_msg.left_foot_contacts = support_data.contact_pts

        self.lc.publish(self.output_channel, contact_msg.encode())

    def run(self):
        while True:
            self.lc.handle()


def main():
    passthrough = ControllerContactPassthrough()
    print "Controller contact passthrough running"
    passthrough.run()


if __name__ == '__main__':
    main()



