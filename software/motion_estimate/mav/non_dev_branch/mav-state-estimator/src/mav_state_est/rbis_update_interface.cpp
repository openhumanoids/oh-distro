#include "rbis_update_interface.hpp"

const char * RBISUpdateInterface::sensor_enum_chars = "igvlfsorx";
const char * RBISUpdateInterface::sensor_enum_strings[] =
  { "ins", "gps", "vicon", "laser", "laser_gpf", "scan_matcher", "optic_flow", "reset", "invalid" };

RBISUpdateInterface::sensor_enum RBISUpdateInterface::sensor_enum_from_char(char sensor_char)
 {
   const char * arg_char = strchr(sensor_enum_chars, sensor_char);
   if (arg_char == NULL) {
     return invalid;
   }

   int sensor_ind = (int) (arg_char - &sensor_enum_chars[0]);
   if (sensor_ind < 0 || sensor_ind > invalid) {
     sensor_ind = invalid;
   }
   return (sensor_enum) sensor_ind;
 }
