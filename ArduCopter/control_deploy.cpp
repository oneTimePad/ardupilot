#include "Copter.h"


bool Copter::deploy_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    return false
#endif

  if (motors->armed()) {
      return false;
  }

  return true;
}


// runs the throw to start controller
// should be called at 100hz or more
void Copter::deploy_run()
{

    /* Throw State Machine
    Throw_Disarmed - motors are off
    Throw_Detecting -  motors are on and we are waiting for the throw
    Throw_Uprighting - the throw has been detected and the copter is being uprighted
    Throw_HgtStabilise - the copter is kept level and  height is stabilised about the target height
    Throw_PosHold - the copter is kept at a constant position and height
    */

    // Don't enter THROW mode if interlock will prevent motors running
    if (!motors->armed() && motors->get_interlock()) {
        // state machine entry is always from a disarmed state
        deploy_state.stage = Deploy_Disarmed;

    } else if (deploy_state.stage == Deploy_Disarmed && motors->armed()) {
        gcs_send_text(MAV_SEVERITY_INFO,"waiting for deployment");
        deploy_state.stage = Deploy_Detecting;

    } else if (deploy_state.stage == Deploy_Detecting && deploy_detected()){
        gcs_send_text(MAV_SEVERITY_INFO,"deploy detected - deploying");
        deploy_state.stage = Deploy_Deploying;

        // Cancel the waiting for throw tone sequence
        AP_Notify::flags.waiting_for_throw = false;

    } else if (deploy_state.stage == Deploy_Deploying) {
        gcs_send_text(MAV_SEVERITY_INFO, "deploy detected - switching to brake mode!");

        // run brake mode controller
        if (!did_brake_time_out()) {
           brake_run();
        } else {
          deploy_state.stage = Deploy_Switch_Mode;
        }

        // Set the auto_arm status to true to avoid a possible automatic disarm caused by selection of an auto mode with throttle at minimum
        set_auto_armed(true);

    } else if (deploy_state.stage == Deploy_Switch_Mode) {
        gcs_send_text(MAV_SEVERITY_INFO, "MAV SUCCESSFULLY DEPLOYED! - switching modes");
        // tell carrier deployment is complete
        gcs_send_message(MSG_DEPLOY_COMPLETE);
        if (!deploy_state.nextmode_attempted) {
          switch (g2.depl_nextmode) {
              case AUTO:
              case GUIDED:
              case RTL:
              case LAND:
              case BRAKE:
                  set_mode((control_mode_t)g2.depl_nextmode.get(), MODE_REASON_DEPLOY_COMPLETE);
                  break;
              default:
                  // do nothing
                  break;
          }
        }
    }

    // Throw State Processing
    switch (deploy_state.stage) {

    case Deploy_Disarmed:
        if (!motors->armed() && deploy_arm) {
           // we received the arming command from the main copter
           copter.init_arm_motors(true);
           send_deploy_cmd = true;
        }
        // reuse some of the throw parameters... for now
        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == 1) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        }

        // demand zero throttle (motors will be stopped anyway) and continually reset the attitude controller
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        break;

    case Deploy_Detecting:

        // prevent motors from rotating before the throw is detected unless enabled by the user
        if (g.throw_motor_start == 1) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        }

        // Hold throttle at zero during the throw and continually reset the attitude controller
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // Play the waiting for throw tone sequence to alert the user
        AP_Notify::flags.waiting_for_throw = true;

        break;

    default:
        break;
    /*
    case Throw_Uprighting:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // demand a level roll/pitch attitude with zero yaw rate
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f, get_smoothing_gain());

        // output 50% throttle and turn off angle boost to maximise righting moment
        attitude_control->set_throttle_out(0.5f, false, g.throttle_filt);

        break;

    case Throw_HgtStabilise:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f, get_smoothing_gain());

        // call height controller
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control->update_z_controller();

        break;

    case Throw_PosHold:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // run loiter controller
        wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0.0f, get_smoothing_gain());

        // call height controller
        pos_control->set_alt_target_from_climb_rate_ff(0.0f, G_Dt, false);
        pos_control->update_z_controller();

        break;
    */
    }

    // log at 10hz or if stage changes
/*    uint32_t now = AP_HAL::millis();
    if ((throw_state.stage != throw_state.prev_stage) || (now - throw_state.last_log_ms) > 100) {
        throw_state.prev_stage = throw_state.stage;
        throw_state.last_log_ms = now;
        float velocity = inertial_nav.get_velocity().length();
        float velocity_z = inertial_nav.get_velocity().z;
        float accel = ins.get_accel().length();
        float ef_accel_z = ahrs.get_accel_ef().z;
        bool throw_detect = (throw_state.stage > Throw_Detecting) || throw_detected();
        bool attitude_ok = (throw_state.stage > Throw_Uprighting) || throw_attitude_good();
        bool height_ok = (throw_state.stage > Throw_HgtStabilise) || throw_height_good();
        bool pos_ok = (throw_state.stage > Throw_PosHold) || throw_position_good();
        Log_Write_Throw(throw_state.stage,
                        velocity,
                        velocity_z,
                        accel,
                        ef_accel_z,
                        throw_detect,
                        attitude_ok,
                        height_ok,
                        pos_ok);
    }*/
}

bool Copter::deploy_detected()
{
    // Check that we have a valid navigation solution
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    if (!filt_status.flags.attitude || !filt_status.flags.horiz_pos_abs || !filt_status.flags.vert_pos) {
        return false;
    }

    // Check for high speed (>500 cm/s)
  //  bool high_speed = inertial_nav.get_velocity().length() > THROW_HIGH_SPEED;

    // check for upwards or downwards trajectory (airdrop) of 50cm/s
    bool changing_height;
//    if (g2.throw_type == ThrowType_Drop) {
    changing_height = inertial_nav.get_velocity().z < -THROW_VERTICAL_SPEED;
//    } else {
//        changing_height = inertial_nav.get_velocity().z > THROW_VERTICAL_SPEED;
//    }

    // Check the vertical acceleraton is greater than 0.25g
    bool free_falling = ahrs.get_accel_ef().z > -0.25 * GRAVITY_MSS;

    // Check if the accel length is < 1.0g indicating that any throw action is complete and the copter has been released
    bool no_throw_action = ins.get_accel().length() < 1.0f * GRAVITY_MSS;

    // High velocity or free-fall combined with increasing height indicate a possible air-drop or throw release
    bool possible_deploy_detected = free_falling && changing_height && no_throw_action;

    // Record time and vertical velocity when we detect the possible throw
    if (possible_deploy_detected && ((AP_HAL::millis() - deploy_state.free_fall_start_ms) > 500)) {
        deploy_state.free_fall_start_ms = AP_HAL::millis();
        deploy_state.free_fall_start_velz = inertial_nav.get_velocity().z;
    }

    // Once a possible throw condition has been detected, we check for 2.5 m/s of downwards velocity change in less than 0.5 seconds to confirm
    bool deploy_condition_confirmed = ((AP_HAL::millis() - deploy_state.free_fall_start_ms < 500) && ((inertial_nav.get_velocity().z - deploy_state.free_fall_start_velz) < -250.0f));

    // start motors and enter the control mode if we are in continuous freefall
    if (deploy_condition_confirmed) {
        return true;
    } else {
        return false;
    }
}

bool Copter::deploy_handle_msg(const mavlink_message_t &msg)
{

  if (control_mode != DEPLOY) {
     return false;
  }

  if (msg.msgid != MAVLINK_MSG_ID_DEPLOY_ARM) {
     return false;
  }

  mavlink_deploy_arm_t packet;
  mavlink_msg_deploy_arm_decode(&msg, &packet);

  if (packet.arm_target != g.sysid_this_mav) {
       return false;
  }



  deploy_arm = true;

  if (send_deploy_cmd) {
     return true;
  }

  return false;
}
