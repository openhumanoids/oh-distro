import ConfigParser
import inspect, os
import datetime

class IRobotHandConfigParser(object):
    motor_encoder_offsets_section_name = 'MotorEncoderOffsets'

    def __init__(self, side):
        self.side = side
        currentfile = inspect.getfile(inspect.currentframe())
        currentdir = os.path.dirname(os.path.abspath(currentfile))

        self.config_file_name = currentdir + '/irobot_hand_config_' + side + '.cfg'
        self.config = ConfigParser.RawConfigParser()

    def load(self):
        self.config.read(self.config_file_name)

    def save(self):
        self.set_metadata()
        with open(self.config_file_name, 'wb') as config_file:
            self.config.write(config_file)

    def clear(self):
        try:
            os.remove(self.config_file_name)
        except OSError:
            pass
        self.load()

    def get_motor_encoder_offset(self, motor_index):
        try:
            section_name = IRobotHandConfigParser.motor_encoder_offsets_section_name
            return self.config.getfloat(section_name, IRobotHandConfigParser.motor_index_name(motor_index))
        except (ConfigParser.NoOptionError, ConfigParser.NoSectionError):
            return 0

    def set_metadata(self):
        now = datetime.datetime.now()
        self.set_including_section("MetaData", "calibration_time", now.isoformat())

    def set_motor_encoder_offset(self, motor_index, value):
        section_name = IRobotHandConfigParser.motor_encoder_offsets_section_name
        key = self.motor_index_name(motor_index)
        self.set_including_section(section_name, key, value)

    def set_including_section(self, section, key, value):
        if self.config.has_section(section):
            self.config.set(section, key, value)
        else:
            self.config.add_section(section)
            self.set_including_section(section, key, value)

    @staticmethod
    def motor_index_name(motor_index):
        return "M%d" % motor_index