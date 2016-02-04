/*
      this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
      smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
      smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
      you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BEETHEFIRST_H
#define BEETHEFIRST_H

#include "Pin.h"
#include "Pwm.h"
#include "../temperaturecontrol/Thermistor.h"
#include <math.h>

#include <string>
using std::string;

class Gcode;
class StreamOutput;
class Thermistor;

namespace mbed {
    class PwmOut;
}

class BEETHEFIRST : public Module {
    public:
		BEETHEFIRST();

        void on_module_loaded();
        void on_main_loop(void *argument);
        void on_config_reload(void* argument);
        void on_gcode_execute(void *argument);
        void on_gcode_received(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_halt(void *arg);
        void on_second_tick(void *argument);


    private:

        bool match_blower_on_gcode(const Gcode* gcode) const;
        bool match_blower_off_gcode(const Gcode* gcode) const;

        int send_gcode(const char* format, ...);

        string		beethefirst_model;

        //Blower stuff
        Pin				*blower_on_pin;
        Pwm				*blower_pwm_pin;
        string			blower_output_on_command;
        string			blower_output_off_command;
        uint16_t 		blower_input_on_command_code;
        uint16_t		blower_input_off_command_code;
        char			blower_input_on_command_letter;
        char			blower_input_off_command_letter;
        uint8_t			blower_subcode;
        bool			blower_state;
        float			blower_value;

        //Extruder Block
        Thermistor		*extruder_block_thermistor;
        float			extruder_temp;
        Pin				*extruder_block_fan_on_pin;
        Pwm				*extruder_block_fan_pwm_pin;
        bool			extruder_block_fan_state;
        float			extruder_block_fan_value;
        bool			extruder_block_fan_auto_mode;
        float			extruder_fan_min_temp;

};

#endif // BEETHEFIRST_H
