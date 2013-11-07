import ConfigParser
import inspect, os

class IRobotHandConfigParser(object):
    motor_encoder_offsets_section_name = 'MotorEncoderOffsets'

    def __init__(self, side):
        self.side = side
        currentfile = inspect.getfile(inspect.currentframe())
        currentdir = os.path.dirname(os.path.abspath(currentfile))
        print currentdir
        
        self.config_file_name = currentdir + '/irobot_hand_config_' + side + '.cfg'
        self.config = ConfigParser.RawConfigParser()

    def load(self):
        self.config.read(self.config_file_name)

    def save(self):
        with open(self.config_file_name, 'wb') as config_file:
            self.config.write(config_file)

    def clear(self):
        os.remove(self.config_file_name)
        self.load()

    def get_motor_encoder_offset(self, motor_index):
        try:
            section_name = IRobotHandConfigParser.motor_encoder_offsets_section_name
            return self.config.getint(section_name, IRobotHandConfigParser.motor_index_name(motor_index))
        except (ConfigParser.NoOptionError, ConfigParser.NoSectionError):
            return 0

    def set_motor_encoder_offset(self, motor_index, value):
        section_name = IRobotHandConfigParser.motor_encoder_offsets_section_name
        if self.config.has_section(section_name):
            self.config.set(section_name, self.motor_index_name(motor_index), value)
        else:
            self.config.add_section(section_name)
            self.set_motor_encoder_offset(motor_index, value)

    @staticmethod
    def motor_index_name(motor_index):
        return "M%d" % motor_index