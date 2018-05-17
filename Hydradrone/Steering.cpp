// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Rover.h"

/*****************************************
* Throttle slew limit
*****************************************/
void Rover::throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate && last_throttle != 0) {
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01f * fabsf(channel_throttle->radio_max - channel_throttle->radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->radio_out = constrain_int16(channel_throttle->radio_out, last_throttle - temp, last_throttle + temp);
    }
}

/*
  check for triggering of start of auto mode
 */
bool Rover::auto_check_trigger(void)
{
    // only applies to AUTO mode
    if (control_mode != AUTO) {
        return true;
    }

    // check for user pressing the auto trigger to off
    if (auto_triggered && g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 1) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("AUTO triggered off"));
        auto_triggered = false;
        return false;
    }

    // if already triggered, then return true, so you don't
    // need to hold the switch down
    if (auto_triggered) {
        return true;
    }

    if (g.auto_trigger_pin == -1 && is_zero(g.auto_kickstart)) {
        // no trigger configured - let's go!
        auto_triggered = true;
        return true;
    }

    if (g.auto_trigger_pin != -1 && check_digital_pin(g.auto_trigger_pin) == 0) {
        gcs_send_text_P(SEVERITY_LOW, PSTR("Triggered AUTO with pin"));
        auto_triggered = true;
        return true;
    }

    if (!is_zero(g.auto_kickstart)) {
        float xaccel = ins.get_accel().x;
        if (xaccel >= g.auto_kickstart) {
            gcs_send_text_fmt(PSTR("Triggered AUTO xaccel=%.1f"), (double)xaccel);
            auto_triggered = true;
            return true;
        }
    }

    return false;
}

/*
  work out if we are going to use pivot steering
 */
bool Rover::use_pivot_steering(void)
{
    if (control_mode >= AUTO && g.skid_steer_out && g.pivot_turn_angle != 0) {
        int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (abs(bearing_error) > g.pivot_turn_angle) {
            return true;
        }
    }
    return false;
}


/*
  test if we are loitering AND should be stopped at a waypoint
*/
bool Rover::in_stationary_loiter()
{
    // Confirm we are in AUTO mode and need to loiter for a time period
    if ((loiter_start_time > 0) && (control_mode == AUTO)) {
        // Check if active loiter is enabled AND we are outside the waypoint loiter radius
        // then the vehicle still needs to move so return false
        if (active_loiter && (wp_distance > g.waypoint_radius)) {
            return false;
        }
        return true;
    }

    return false;
}


/*
  calculate the throtte for auto-throttle modes
 */
void Rover::calc_throttle(float target_speed)
{
    // If not autostarting OR we are loitering at a waypoint
    // then set the throttle to minimum
    if (!auto_check_trigger() || in_stationary_loiter()) {
        channel_throttle->servo_out = g.throttle_min.get();
        return;
    }

    float throttle_base = (fabsf(target_speed) / g.speed_cruise) * g.throttle_cruise;
    int throttle_target = throttle_base + throttle_nudge;

    /*
      reduce target speed in proportion to turning rate, up to the
      SPEED_TURN_GAIN percentage.
    */
    float steer_rate = fabsf(lateral_acceleration / (g.turn_max_g*GRAVITY_MSS));
    steer_rate = constrain_float(steer_rate, 0.0f, 1.0f);

    // use g.speed_turn_gain for a 90 degree turn, and in proportion
    // for other turn angles
    int32_t turn_angle = wrap_180_cd(next_navigation_leg_cd - ahrs.yaw_sensor);
    float speed_turn_ratio = constrain_float(fabsf(turn_angle / 9000.0f), 0, 1);
    float speed_turn_reduction = (100 - g.speed_turn_gain) * speed_turn_ratio * 0.01f;

    float reduction = 1.0f - steer_rate*speed_turn_reduction;

    if (control_mode >= AUTO && wp_distance <= g.speed_turn_dist) {
        // in auto-modes we reduce speed when approaching waypoints
        float reduction2 = 1.0f - speed_turn_reduction;
        if (reduction2 < reduction) {
            reduction = reduction2;
        }
    }

    // reduce the target speed by the reduction factor
    target_speed *= reduction;

    groundspeed_error = fabsf(target_speed) - ground_speed;

    throttle = throttle_target + (g.pidSpeedThrottle.get_pid(groundspeed_error * 100) / 100);

    // also reduce the throttle by the reduction factor. This gives a
    // much faster response in turns
    throttle *= reduction;

    if (in_reverse) {
        channel_throttle->servo_out = constrain_int16(-throttle, -g.throttle_max, -g.throttle_min);
    } else {
        channel_throttle->servo_out = constrain_int16(throttle, g.throttle_min, g.throttle_max);
    }

    if (!in_reverse && g.braking_percent != 0 && groundspeed_error < -g.braking_speederr) {
        // the user has asked to use reverse throttle to brake. Apply
        // it in proportion to the ground speed error, but only when
        // our ground speed error is more than BRAKING_SPEEDERR.
        //
        // We use a linear gain, with 0 gain at a ground speed error
        // of braking_speederr, and 100% gain when groundspeed_error
        // is 2*braking_speederr
        float brake_gain = constrain_float(((-groundspeed_error)-g.braking_speederr)/g.braking_speederr, 0, 1);
        int16_t braking_throttle = g.throttle_max * (g.braking_percent * 0.01f) * brake_gain;
        channel_throttle->servo_out = constrain_int16(-braking_throttle, -g.throttle_max, -g.throttle_min);

        // temporarily set us in reverse to allow the PWM setting to
        // go negative
        set_reverse(true);
    }

    if (use_pivot_steering()) {
        channel_throttle->servo_out = 0;
    }
}

/*****************************************
 * Calculate desired turn angles (in medium freq loop)
 *****************************************/

void Rover::calc_lateral_acceleration()
{
    switch (control_mode) {
    case AUTO:
        // If we have reached the waypoint previously navigate
        // back to it from our current position
        if (previously_reached_wp && (loiter_duration > 0)) {
            nav_controller->update_waypoint(current_loc, next_WP);
        } else {
            nav_controller->update_waypoint(prev_WP, next_WP);
        }
        break;

    case RTL:
    case GUIDED:
    case STEERING:
        nav_controller->update_waypoint(current_loc, next_WP);
        break;
    default:
        return;
    }

	// Calculate the required turn of the wheels

    // negative error = left turn
	// positive error = right turn
    lateral_acceleration = nav_controller->lateral_acceleration();
    if (use_pivot_steering()) {
        int16_t bearing_error = wrap_180_cd(nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100;
        if (bearing_error > 0) {
            lateral_acceleration = g.turn_max_g*GRAVITY_MSS;
        } else {
            lateral_acceleration = -g.turn_max_g*GRAVITY_MSS;
        }
    }
}

/*
  calculate steering angle given lateral_acceleration
 */
void Rover::calc_nav_steer()
{
    // check to see if the rover is loitering
    if (in_stationary_loiter()) {
        channel_steer->servo_out = 0;
        return;
    }

    // add in obstacle avoidance
    lateral_acceleration += (obstacle.turn_angle/45.0f) * g.turn_max_g;

    // constrain to max G force
    lateral_acceleration = constrain_float(lateral_acceleration, -g.turn_max_g*GRAVITY_MSS, g.turn_max_g*GRAVITY_MSS);

    channel_steer->servo_out = steerController.get_steering_out_lat_accel(lateral_acceleration);
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
void Rover::set_servos(void)
{
  channel_FL->output();
  channel_FR->output();
  channel_RL->output();
  channel_RR->output();
}

void Rover::radio_mode_one()
{
    RC_Channel::set_pwm_all();
    int32_t mindg = channel_FL->radio_min;
    int32_t maxdg = channel_FL->radio_max;
    float scale_dg = 1000.0/abs(maxdg - mindg);
    float commande_dg = (channel_FL->radio_in - mindg)*scale_dg*0.99;
    int32_t c_dg = commande_dg - 500;

    int32_t minaa = channel_FR->radio_min;
    int32_t maxaa = channel_FR->radio_max;
    float scale_aa = 1000.0/abs(maxaa - minaa);
    float commande_aa = (channel_FR->radio_in - minaa)*scale_aa*0.99;
    int32_t c_aa = 500 - commande_aa;

    float dir = 0;
    if(c_aa<50 && c_aa>-50 && c_dg > -50 && c_dg <50)
    {
      channel_FL->radio_out = 1000;
      channel_FR->radio_out = 1000;
      channel_RL->radio_out = 1000;
      channel_RR->radio_out = 1000;
    }
    else
    {
    dir = atan2( c_dg, c_aa);
    if(dir<0)
    {
      dir=dir+2*M_PI;
    }
    direction(dir);
    /*channel_FL->radio_out = 1000 + c_dg;
    channel_FR->radio_out = 1000 + c_aa;
    channel_RL->radio_out = 1000+ dir*360/(2*M_PI);
    channel_RR->radio_out = 1000;*/
  }


}

void Rover::calc_direction()
{
    direction_to_go = ((nav_controller->target_bearing_cd() - ahrs.yaw_sensor) / 100)*2*M_PI/360;
}

void Rover::direction(float command_angle)
{

   float coefFL;
   float coefFR;
   float coefRL;
   float coefRR;

   coefFR = cos(command_angle + M_PI/4);
   coefFL = sin(command_angle + M_PI/4);
   coefRR = -sin(command_angle + M_PI/4);
   coefRL = -cos(command_angle + M_PI/4);

   if(coefFR < 0)
   {
     hal.rcout->write(pinFR, 500);
   }
   else
   {
     hal.rcout->write(pinFR, 1500);
   }

   if(coefFL < 0)
   {
     hal.rcout->write(pinFL, 500);
   }
   else
   {
     hal.rcout->write(pinFL, 1500);
   }

   if(coefRL < 0)
   {
     hal.rcout->write(pinRL, 500);
   }
   else
   {
     hal.rcout->write(pinRL, 1500);
   }

   if(coefRR < 0)
   {
     hal.rcout->write(pinRR, 500);
   }
   else
   {
     hal.rcout->write(pinRR, 1500);
   }
   channel_FL->radio_out = 1000 + abs(coefFL*1000);
   channel_FR->radio_out = 1000 + abs(coefFR*1000);
   channel_RL->radio_out = 1000 + abs(coefRL*1000);
   channel_RR->radio_out = 1000 + abs(coefRR*1000);
}


void Rover::radio_mode_two()
{
  RC_Channel::set_pwm_all();
  float scale_dg = 1000/(channel_FL->radio_max - channel_FL->radio_min);
  float commande_dg = (channel_FL->radio_in - channel_FL->radio_min)*scale_dg;
  int32_t c_dg = commande_dg;

  float scale_aa = 1000/(channel_FR->radio_max - channel_FR->radio_min);
  float commande_aa = (channel_FR->radio_in - channel_FR->radio_min)*scale_aa;
  int32_t c_aa = commande_aa;

  if(c_aa<200)
  {
      direction(0);
  }
  else if(c_aa>800)
  {
      direction(M_PI);
  }
  else
  {
    if(c_dg<200)
    {
      turn_right();
    }
    else if(c_dg>800)
    {
      turn_left();
    }
    else
    {
      channel_FL->radio_out = 1000;
      channel_FR->radio_out = 1000;
      channel_RL->radio_out = 1000;
      channel_RR->radio_out = 1000;
    }
  }
}

void Rover::turn_right()
{
  hal.rcout->write(pinFL, 1500);
  hal.rcout->write(pinFR, 1500);
  hal.rcout->write(pinRL, 1500);
  hal.rcout->write(pinRR, 1500);
  channel_FL->radio_out = 1500;
  channel_FR->radio_out = 1000;
  channel_RL->radio_out = 1000;
  channel_RR->radio_out = 1500;
}

void Rover::turn_left()
{
  hal.rcout->write(pinFL, 500);
  hal.rcout->write(pinFR, 1500);
  hal.rcout->write(pinRL, 500);
  hal.rcout->write(pinRR, 1500);
  channel_FL->radio_out = 1500;
  channel_FR->radio_out = 1000;
  channel_RL->radio_out = 1000;
  channel_RR->radio_out = 1500;
}

void Rover::stop()
{
  hal.rcout->write(pinFL, 1500);
  hal.rcout->write(pinFR, 1500);
  hal.rcout->write(pinRL, 1500);
  hal.rcout->write(pinRR, 1500);
  channel_FL->radio_out = 1000;
  channel_FR->radio_out = 1000;
  channel_RL->radio_out = 1000;
  channel_RR->radio_out = 1000;
}
