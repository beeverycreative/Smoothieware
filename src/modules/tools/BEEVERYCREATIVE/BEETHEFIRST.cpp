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
#include "Robot.h"

#include "libs/FileStream.h"
#include "libs/AppendFileStream.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"

#include "PwmOut.h"

#include "MRI_Hooks.h"

#define beethefirst_checksum				CHECKSUM("beethefirst")
#define enable_checksum						CHECKSUM("enable")
#define model_checksum						CHECKSUM("model")
#define blower_checksum						CHECKSUM("blower")
#define extruder_block_fan_checksum			CHECKSUM("extruder_block_fan")
#define extruder_block_thermistor_checksum	CHECKSUM("extruder_block_thermistor")


#define on_command_checksum					CHECKSUM("on_command")
#define off_command_checksum				CHECKSUM("off_command")
#define enable_pin_checksum					CHECKSUM("enable_pin")
#define pwm_pin_checksum					CHECKSUM("pwm_pin")
#define startup_state						CHECKSUM("startup_state")



BEETHEFIRST::BEETHEFIRST(){}

void BEETHEFIRST::on_halt(void *argument)
{

}

void BEETHEFIRST::on_module_loaded()
{
	// Settings
	if( THEKERNEL->config->value(beethefirst_checksum, enable_checksum )->as_bool() == true )
	{
		THEKERNEL->streams->printf("BEETHEFIRST Module Enabled\n");

		this->register_for_event(ON_HALT);
		this->register_for_event(ON_MAIN_LOOP);
		//this->register_for_event(ON_GCODE_EXECUTE);
		this->register_for_event(ON_GCODE_RECEIVED);
		this->register_for_event(ON_GET_PUBLIC_DATA);
		this->register_for_event(ON_SET_PUBLIC_DATA);
		this->register_for_event(ON_SECOND_TICK);

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

	this->beethefirst_model = THEKERNEL->config->value(beethefirst_checksum, model_checksum )->by_default("")->as_string();

	/*
	 * 			Load Blower Config
	 */
	this->blower_on_pin = new Pin();
	this->blower_on_pin->from_string(THEKERNEL->config->value(beethefirst_checksum, blower_checksum, enable_pin_checksum)->by_default("nc")->as_string())->as_output();
	this->blower_state = THEKERNEL->config->value(beethefirst_checksum, blower_checksum, startup_state)->by_default(false)->as_bool();
	this->blower_pwm_pin = new Pwm();
	this->blower_pwm_pin->from_string(THEKERNEL->config->value(beethefirst_checksum, blower_checksum, pwm_pin_checksum )->by_default("nc")->as_string())->as_output();
	this->blower_pwm_pin->connected();
	this->blower_pwm_pin->max_pwm(255);
	this->blower_pwm_pin->set(false);
	THEKERNEL->slow_ticker->attach(1000, this->blower_pwm_pin, &Pwm::on_tick);
	//this->blower_output_on_command = THEKERNEL->config->value(beethefirst_checksum, blower_checksum, on_command_checksum )->by_default("")->as_string();
	//this->blower_output_off_command = THEKERNEL->config->value(beethefirst_checksum, blower_checksum, off_command_checksum )->by_default("")->as_string();

	/*
	 * 			Load Extruder Block Config
	 */
	this->extruder_block_thermistor = new Thermistor();
	this->extruder_block_thermistor->UpdateConfig(beethefirst_checksum, extruder_block_thermistor_checksum);
	this->extruder_block_fan_on_pin = new Pin();
	this->extruder_block_fan_on_pin->from_string(THEKERNEL->config->value(beethefirst_checksum, extruder_block_fan_checksum, enable_pin_checksum)->by_default("nc")->as_string())->as_output();
	this->extruder_block_fan_state = THEKERNEL->config->value(beethefirst_checksum, extruder_block_fan_checksum, startup_state)->by_default(false)->as_bool();
	this->extruder_block_fan_on_pin->set(this->extruder_block_fan_state);
	this->extruder_block_fan_pwm_pin = new Pwm();
	this->extruder_block_fan_pwm_pin->from_string(THEKERNEL->config->value(beethefirst_checksum, extruder_block_fan_checksum, pwm_pin_checksum )->by_default("nc")->as_string())->as_output();
	this->extruder_block_fan_pwm_pin->connected();
	this->extruder_block_fan_pwm_pin->max_pwm(255);
	this->extruder_block_fan_pwm_pin->set(false);
	THEKERNEL->slow_ticker->attach(1000, this->extruder_block_fan_pwm_pin, &Pwm::on_tick);
	this->extruder_block_fan_value = 0.0;
	this->extruder_block_fan_auto_mode = false;


	/*
	 * 			Parse loaded config
	 */


	this->blower_on_pin->set(this->blower_state);
	this->blower_value = 0.0;

}

void BEETHEFIRST::on_gcode_execute(void *argument)
{

}

void BEETHEFIRST::on_gcode_received(void *argument)
{
	Gcode *gcode = static_cast<Gcode *>(argument);

	if(gcode->has_g)
	{
		switch(gcode->g)
		{
		//G28 - Home axis
		case 28:
		{
			//If calibration procedure is running, cancel it
			if(this->currentCalibrationState != -1) this->currentCalibrationState = 0;

		}
		break;
		//G131 - Start Calibration
		case 131:
		{
			float initial_z = 2;
			this->currentCalibrationState = -1;
			send_gcode("G28");

			if(gcode->has_letter('Z'))
			{
				initial_z = gcode->get_value('Z');
			}

			//THEKERNEL->conveyor->wait_for_empty_queue();
			//send_gcode("M204 S400 Z400");
			send_gcode("G0 X0 Y65 Z%f F10000",initial_z);
			//send_gcode("M204 S1000 Z1000");
			this->currentCalibrationState = 0;
		}
		break;
		//G132 - Proceed to next Calibration step
		case 132:
		{
			switch(this->currentCalibrationState)
			{
			case 0:
			{
				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("M306 Z0");
				send_gcode("M500");

				THECONVEYOR->wait_for_idle();
				THEROBOT->pop_state();

				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("G0 Z10 F1000");
				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("G0 X-30 Y-65 F15000");
				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("G0 Z0 F1000");

				this->currentCalibrationState ++;
			}
			break;
			case 1:
			{
				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("G0 Z10 F1000");
				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("G0 X30 Y-65 F15000");
				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("G0 Z0 F1000");

				this->currentCalibrationState ++;
			}
			break;
			case 2:
			{
				//THEKERNEL->conveyor->wait_for_empty_queue();
				send_gcode("G28");
			}
			break;
			default:
			{

			}
			}
		}
		break;
		}
	}
	else if(gcode->has_m)
	{
		switch(gcode->m)
		{
		//M32 - Start SD Print
		case 32:
		{
			this->extruder_block_fan_auto_mode = true;
			this->extruder_block_fan_value = 0.0;
		}
		break;

		//M106 - Turn Blower ON
		case 106:
		{
			this->blower_on_pin->set(true);
			if(gcode->has_letter('S')) {
				int v = (gcode->get_int('S') * this->blower_pwm_pin->max_pwm()) / 255; // scale by max_pwm so input of 255 and max_pwm of 128 would set value to 128
				if(v != this->blower_pwm_pin->get_pwm()){ // optimize... ignore if already set to the same pwm
					// drain queue
					//THEKERNEL->conveyor->wait_for_empty_queue();
					this->blower_pwm_pin->pwm(v);
					this->blower_state = (v > 0);
				}
			} else {
				// drain queue
				//THEKERNEL->conveyor->wait_for_empty_queue();
				this->blower_pwm_pin->pwm(this->blower_value);
				this->blower_state = (this->blower_value > 0);
			}
		}
		break;

		//M107 - Turn Blower Off
		case 107:
		{
			this->blower_on_pin->set(false);
			// drain queue
			//THEKERNEL->conveyor->wait_for_empty_queue();
			this->blower_state = false;
			this->blower_pwm_pin->set(false);
		}
		break;

		//M126 - Turn Blower ON
		case 126:
		{
			this->extruder_block_fan_on_pin->set(true);
			if(gcode->has_letter('S'))
			{
				this->extruder_block_fan_auto_mode = false;
				int v = (gcode->get_int('S') * this->extruder_block_fan_pwm_pin->max_pwm()) / 255; // scale by max_pwm so input of 255 and max_pwm of 128 would set value to 128
				if(v != this->extruder_block_fan_pwm_pin->get_pwm()){ // optimize... ignore if already set to the same pwm
					// drain queue
					//THEKERNEL->conveyor->wait_for_empty_queue();
					this->extruder_block_fan_pwm_pin->pwm(v);
					this->extruder_block_fan_state = (v > 0);
				}
			} else {
				this->extruder_block_fan_auto_mode = true;
				this->extruder_block_fan_value = 0.0;
			}
		}
		break;

		//M127 - Turn Blower Off
		case 127:
		{
			this->extruder_block_fan_on_pin->set(false);
			// drain queue
			//THEKERNEL->conveyor->wait_for_empty_queue();
			this->extruder_block_fan_state = false;
			this->extruder_block_fan_pwm_pin->set(false);
			this->extruder_block_fan_auto_mode = false;
			this->extruder_block_fan_value = 0.0;

		}
		break;

		//M640 - Pause Print
		case 640:
		{
			__disable_irq();
			string upload_filename;
			FILE *upload_fd;
			StreamOutput* upload_stream{nullptr};

			upload_filename = "/sd/BEEVERYCREATIVE.txt";

			string single_command;
			single_command = "M640\n";
			static int cnt = 0;

			if(fwrite(single_command.c_str(), 1, single_command.size(), upload_fd) != single_command.size()) {
				// error writing to file
				THEKERNEL->streams->printf("Error:error writing to file.\r\n");
				fclose(upload_fd);
				upload_fd = NULL;
				break;

			} else {
				cnt += single_command.size();
				if (cnt > 400) {
					// HACK ALERT to get around fwrite corruption close and re open for append
					fclose(upload_fd);
					upload_fd = fopen(upload_filename.c_str(), "a");
					cnt = 0;
				}
				THEKERNEL->streams->printf("ok\r\n");
				//printf("uploading file write ok\n");
			}
			__enable_irq();

			THEKERNEL->streams->printf("Settings Stored to BEEVERYCREATIVE.txt\r\nok\r\n");
		}
		break;

		//M701 - Load Filament
		case 701:
		{
			// drain queue
			//THEKERNEL->conveyor->wait_for_empty_queue();
			this->send_gcode("G0 E65 F300");
			this->send_gcode("G90");
			this->send_gcode("G92 E0");
		}
		break;

		//M702 - Unoad Filament
		case 702:
		{
			// drain queue
			//THEKERNEL->conveyor->wait_for_empty_queue();
			this->send_gcode("G91");
			this->send_gcode("G0 E15 F300");
			this->send_gcode("G0 E-23 F1000");
			this->send_gcode("G0 E25 F800");
			this->send_gcode("G0 E-30 F2000");
			this->send_gcode("G0 E-50 F200");
			this->send_gcode("G90");
			this->send_gcode("G92 E0");
			//THEKERNEL->conveyor->wait_for_empty_queue();
		}
		break;

		//M1400 - Get BEETHEFIRST PLUS Temperatures
		case 1400:
		{
			THEKERNEL->streams->printf("Extruder Block Temp: %f Fan Speed: %f\n",this->extruder_temp,this->extruder_block_fan_value);
		}
		break;

		// you can have any number of case statements.
		default : //Optional
		{

		}
		}
	}
}

void BEETHEFIRST::on_get_public_data(void *argument)
{

}

void BEETHEFIRST::on_set_public_data(void *argument)
{

}

/***************************************************************************************************
 *
 * 								on_second_tick
 *
 *
 *
 ***************************************************************************************************/
void BEETHEFIRST::on_second_tick(void *argument)
{
	this->extruder_temp = this->extruder_block_thermistor->get_temperature();
	float extruder_fan_speed = 0.0;

	if(this->extruder_block_fan_auto_mode == true)
	{
		extruder_fan_speed = this->extruder_temp * 22.3125 - 704.4375;

		if(this->extruder_block_fan_value == 0 && this->extruder_temp < 34)
		{
			extruder_fan_speed = 0;

		}

		if(extruder_fan_speed != 0)
		{
			if(this->extruder_temp < 32) extruder_fan_speed = 0;

			//if(extruder_fan_speed < 0) extruder_fan_speed = 0;
			if(extruder_fan_speed < 30 && this->extruder_temp < 32) extruder_fan_speed = 0;
			else if(extruder_fan_speed > 255) extruder_fan_speed = 255;
		}

	}


	if(this->extruder_block_fan_value != extruder_fan_speed)
	{
		if (!THEKERNEL->conveyor->is_queue_full())
		{
			this->extruder_block_fan_pwm_pin->pwm(extruder_fan_speed);
					this->extruder_block_fan_state = (extruder_fan_speed > 0);
					this->extruder_block_fan_value = extruder_fan_speed;
		}

	}


	//THEKERNEL->streams->printf("Extruder Temp: %f\n",extruder_temp);
	//float block_fan_speed = extruder_temp*5.77 - 281.15;
	//this->extruder_block_fan_pwm_pin->pwm(block_fan_speed);
	//this->extruder_block_fan_pwm_pin->pwm(255);
}

/* send a formatted Gcode line */
int BEETHEFIRST::send_gcode(const char* format, ...)
{
	// handle variable arguments
	va_list args;
	va_start(args, format);
	// make the formatted string
	char line[32]; // max length for an gcode line
	int n = vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	// debug, print the gcode sended
	//THEKERNEL->streams->printf(">>> %s\r\n", line);
	// make gcode object and send it (right way)
	Gcode gc(line, &(StreamOutput::NullStream));
	THEKERNEL->call_event(ON_GCODE_RECEIVED, &gc);
	// return the gcode srting length
	return n;
}



