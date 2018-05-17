// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/*
  allow for runtime change of control channel ordering
 */
void Rover::set_control_channels(void)
{
    channel_FL = RC_Channel::rc_channel(rcmap.roll()-1);
    channel_FR = RC_Channel::rc_channel(rcmap.pitch()-1);
    channel_RL = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_RR = RC_Channel::rc_channel(rcmap.yaw()-1);


	// set rc channel ranges
	channel_steer->set_angle(SERVO_MAX);
	channel_throttle->set_angle(100);

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed.
    hal.rcout->set_esc_scaling(channel_throttle->radio_min, channel_throttle->radio_max);
}

void Rover::init_rc_in()
{
	// set rc dead zones
	channel_steer->set_default_dead_zone(30);
	channel_throttle->set_default_dead_zone(30);

	//set auxiliary ranges
    update_aux();
}

void Rover::init_rc_out()
{
    RC_Channel::rc_channel(CH_1)->enable_out();
    RC_Channel::rc_channel(CH_2)->enable_out();
    RC_Channel::rc_channel(CH_3)->enable_out();
    RC_Channel::rc_channel(CH_4)->enable_out();
    hal.rcout->enable_ch(4);
    hal.rcout->enable_ch(5);
    hal.rcout->enable_ch(6);
    hal.rcout->enable_ch(7);
  //  RC_Channel::output_trim_all();

    // setup PWM values to send if the FMU firmware dies
  //  RC_Channel::setup_failsafe_trim_all();
}

void Rover::read_radio()
{
      RC_Channel::set_pwm_all();
}

void Rover::control_failsafe(uint16_t pwm)
{
	if (!g.fs_throttle_enabled) {
        // no throttle failsafe
		return;
    }

	// Check for failsafe condition based on loss of GCS control
	if (rc_override_active) {
        failsafe_trigger(FAILSAFE_EVENT_RC, (millis() - failsafe.rc_override_timer) > 1500);
	} else if (g.fs_throttle_enabled) {
        bool failed = pwm < (uint16_t)g.fs_throttle_value;
        if (hal.scheduler->millis() - failsafe.last_valid_rc_ms > 2000) {
            failed = true;
        }
        failsafe_trigger(FAILSAFE_EVENT_THROTTLE, failed);
	}
}

void Rover::trim_control_surfaces()
{
	read_radio();
	// Store control surface trim values
	// ---------------------------------
    if (channel_steer->radio_in > 1400) {
		channel_steer->radio_trim = channel_steer->radio_in;
        // save to eeprom
        channel_steer->save_eeprom();
    }
}

void Rover::trim_radio()
{
	for (int y = 0; y < 30; y++) {
		read_radio();
	}
    trim_control_surfaces();
}
