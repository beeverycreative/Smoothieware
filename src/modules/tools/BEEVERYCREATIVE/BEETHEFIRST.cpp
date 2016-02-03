#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/SerialMessage.h"
#include <math.h>
#include "BEETHEFIRST.h"
#include "libs/Pin.h"
#include "modules/robot/Conveyor.h"
#include "PublicDataRequest.h"
#include "SwitchPublicAccess.h"
#include "SlowTicker.h"
#include "Config.h"
#include "Gcode.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

#include "PwmOut.h"

#include "MRI_Hooks.h"

#define beethefirst_checksum				CHECKSUM("beethefirst")
#define enable_checksum						CHECKSUM("enable")
#define model_checksum						CHECKSUM("model")
#define blower_on_command_checksum			CHECKSUM("blower_on_command")
#define blower_off_command_checksum			CHECKSUM("blower_off_command")
#define blower_enable_pin_checksum			CHECKSUM("blower_enable_pin")
#define blower_pwm_pin_checksum				CHECKSUM("blower_pwm_pin")
#define blower_startup_state				CHECKSUM("blower_startup_state")

BEETHEFIRST::BEETHEFIRST(){}

void BEETHEFIRST::on_halt(void *argument)
{

}

void BEETHEFIRST::on_module_loaded()
{
	this->register_for_event(ON_HALT);
	this->register_for_event(ON_MAIN_LOOP);
	this->register_for_event(ON_GCODE_EXECUTE);
	this->register_for_event(ON_GCODE_RECEIVED);
	this->register_for_event(ON_GET_PUBLIC_DATA);
	this->register_for_event(ON_SET_PUBLIC_DATA);

	// Settings
	if( THEKERNEL->config->value(beethefirst_checksum, enable_checksum )->as_bool() == true )
	{
		THEKERNEL->streams->printf("BEETHEFIRST Module Enabled\n");
		this->on_config_reload(this);
	} else {
		THEKERNEL->streams->printf("BEETHEFIRST Module Disabled\n");
	}
}

void BEETHEFIRST::on_main_loop(void *argument)
{

}


/***************************************************************************************************
 *
 * 								on_config_reload
 *
 *
 *
 ***************************************************************************************************/
void BEETHEFIRST::on_config_reload(void *argument)
{
	/*
	 * 			Load Config
	 */
	this->beethefirst_model = THEKERNEL->config->value(beethefirst_checksum, model_checksum )->by_default("")->as_string();

	std::string blower_input_on_command = THEKERNEL->config->value(beethefirst_checksum,blower_on_command_checksum)->by_default("")->as_string();
	std::string blower_input_off_command = THEKERNEL->config->value(beethefirst_checksum,blower_off_command_checksum)->by_default("")->as_string();


	this->blower_on_pin = new Pin();
	this->blower_on_pin->from_string(THEKERNEL->config->value(beethefirst_checksum, blower_enable_pin_checksum)->by_default("nc")->as_string())->as_output();

	this->blower_state = THEKERNEL->config->value(beethefirst_checksum, blower_startup_state)->by_default(false)->as_bool();

	this->blower_pwm_pin = new Pwm();
	this->blower_pwm_pin->from_string(THEKERNEL->config->value(beethefirst_checksum, blower_pwm_pin_checksum )->by_default("nc")->as_string())->as_output();
	this->blower_pwm_pin->connected();
	this->blower_pwm_pin->max_pwm(255);
	this->blower_pwm_pin->set(false);
	THEKERNEL->slow_ticker->attach(1000, this->blower_pwm_pin, &Pwm::on_tick);


	this->blower_output_on_command = THEKERNEL->config->value(beethefirst_checksum, blower_on_command_checksum )->by_default("")->as_string();
	this->blower_output_off_command = THEKERNEL->config->value(beethefirst_checksum, blower_off_command_checksum )->by_default("")->as_string();



	/*
	 * 			Parse loaded config
	 */


	this->blower_on_pin->set(this->blower_state);
	this->blower_value = 0.0;


	// Set the on/off command codes, Use GCode to do the parsing
	blower_input_on_command_letter= 0;
	blower_input_off_command_letter= 0;

	if(!blower_output_on_command.empty())
	{
		Gcode gc(blower_output_on_command, NULL);
		if(gc.has_g)
		{
			blower_input_on_command_letter = 'G';
			blower_input_on_command_code = gc.g;
			THEKERNEL->streams->printf("Blower ON Gcode: G%u\n",gc.g);
		}
		else if(gc.has_m)
		{
			blower_input_on_command_letter = 'M';
			blower_input_on_command_code = gc.m;
			THEKERNEL->streams->printf("Blower ON Mcode: M%u\n",gc.m);
		}
	}

	if(!blower_output_off_command.empty())
	{
		Gcode gc(blower_output_off_command, NULL);
		if(gc.has_g)
		{
			blower_input_off_command_letter = 'G';
			blower_input_off_command_code = gc.g;
			THEKERNEL->streams->printf("Blower OFF Gcode: G%u\n",gc.g);
		}
		else if(gc.has_m)
		{
			blower_input_off_command_letter = 'M';
			blower_input_off_command_code = gc.m;
			THEKERNEL->streams->printf("Blower OFF Mcode: M%u\n",gc.m);
		}
	}

}

bool BEETHEFIRST::match_blower_on_gcode(const Gcode *gcode) const
{
    bool b= ((blower_input_on_command_letter == 'M' && gcode->has_m && gcode->m == blower_input_on_command_code) ||
            (blower_input_on_command_letter == 'G' && gcode->has_g && gcode->g == blower_input_on_command_code));

    return (b && gcode->subcode == this->blower_subcode);
}

bool BEETHEFIRST::match_blower_off_gcode(const Gcode *gcode) const
{
	bool b= ((blower_input_off_command_letter == 'M' && gcode->has_m && gcode->m == blower_input_off_command_code) ||
	            (blower_input_off_command_letter == 'G' && gcode->has_g && gcode->g == blower_input_off_command_code));
    return (b && gcode->subcode == this->blower_subcode);
}

void BEETHEFIRST::on_gcode_execute(void *argument)
{

}

void BEETHEFIRST::on_gcode_received(void *argument)
{
	Gcode *gcode = static_cast<Gcode *>(argument);
	char gcLetter;
	if(gcode->has_g)
		{
			gcLetter = 'G';
			THEKERNEL->streams->printf("Received gcode: %c%u\n",gcLetter,gcode->g);
		}
	else if(gcode->has_m)
		{
			gcLetter = 'M';
			THEKERNEL->streams->printf("Received gcode: %c%u\n",gcLetter,gcode->m);
		}


	if (!(match_blower_on_gcode(gcode) || match_blower_off_gcode(gcode))) {
		return;
	}

	if(match_blower_on_gcode(gcode))
	{
		this->blower_on_pin->set(true);
		if(gcode->has_letter('S')) {
			int v = (gcode->get_int('S') * blower_pwm_pin->max_pwm()) / 255; // scale by max_pwm so input of 255 and max_pwm of 128 would set value to 128
			if(v != this->blower_pwm_pin->get_pwm()){ // optimize... ignore if already set to the same pwm
				// drain queue
				THEKERNEL->conveyor->wait_for_empty_queue();
				this->blower_pwm_pin->pwm(v);
				this->blower_state = (v > 0);
			}
		} else {
            // drain queue
            THEKERNEL->conveyor->wait_for_empty_queue();
            this->blower_pwm_pin->pwm(this->blower_value);
            this->blower_state = (this->blower_value > 0);
        }
	}
	else if(match_blower_off_gcode(gcode))
	{
		this->blower_on_pin->set(false);
		// drain queue
		THEKERNEL->conveyor->wait_for_empty_queue();
		this->blower_state = false;
		this->blower_pwm_pin->set(false);

	}
}

void BEETHEFIRST::on_get_public_data(void *argument)
{

}

void BEETHEFIRST::on_set_public_data(void *argument)
{

}




