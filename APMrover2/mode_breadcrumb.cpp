#include "mode.h"
#include "Rover.h"


bool ModeBreadcrumb::_enter()
{
    //don't call enter again if _crumbs exists
    if (_crumbs != nullptr) {
        return false;
    }

    // create semaphore
    _crumbs_sem = hal.util->new_semaphore();
    if (_crumbs_sem == nullptr) {
        return false;
    }

    _current_crumb = nullptr;

    // allocate crumbs queue
    _crumbs = (Location*)calloc(crumbs_max, sizeof(Location));

    _crumbs_tail = 0;
    _crumbs_head = 0;

    lonely_mode = nullptr;

    //reuse guided mode
    if (!ModeGuided::_enter()) {
        free(_crumbs);
        return false;
    }

    return true;
}

void ModeBreadcrumb::_exit()
{
    //reset lonely mode in case we go back into breadcrumb mode
    lonely_mode = nullptr;

    _current_crumb = nullptr;

    if (_crumbs != nullptr) {
        free(_crumbs);
        //stop usage of freed data
        _crumbs = nullptr;
    }
}

void ModeBreadcrumb::run_lonely_mode()
{
    if (lonely_mode == nullptr) {
        rover.mode_hold.enter();
        lonely_mode = &rover.mode_hold;

        gcs().send_text(MAV_SEVERITY_INFO, "Breadcrumb: Lonely; %s", lonely_mode->name4());
    }

    lonely_mode->update();
}

void ModeBreadcrumb::update()
{


    float distance_to_breadcrumb = ModeGuided::get_distance_to_destination();

    //we have or are going to a breadcrumb
    //get_distance_to_destination returns exactly 0.0f when destination is reached
    if (_current_crumb != nullptr && distance_to_breadcrumb != 0.0f) {

        Vector3f to_vehicle = location_3d_diff_NED(rover.current_loc, target_loc);
        const float distance_to_vehicle = to_vehicle.length();

        //assign to default value (won't be used)
        float desired_speed = -1.0f;

        //maintain at greater than distance_to_stop from target
        if (fabsf(distance_to_vehicle) > distance_to_stop) {
            //we need to speed up
            to_vehicle *= closure_speed * 100; // m/s to cm/s (which set_desired_speed takes)
            to_vehicle+=target_vel;

            desired_speed = to_vehicle.length();
        } else if (fabsf(distance_to_vehicle) < distance_to_stop) {
            //too close. stop until target is far enough away
            desired_speed = 0.0f;
        }

        //don't change speed if we are distance_to_stop away;
        if (desired_speed != -1.0f) {
            Mode::set_desired_speed(desired_speed);
        }

        // re-use guided mode with set waypoint
        ModeGuided::update();

    } else {

        if (!_crumbs_sem->take_nonblocking()) {
            return;
        }

        //accounts for possible looping around when calculating size
        uint16_t crumbs_queue_size = (_crumbs_tail >= _crumbs_head) ? (_crumbs_tail - _crumbs_head) : crumbs_max - (_crumbs_head-_crumbs_tail);

        if (crumbs_queue_size == 0) {
            _current_crumb = nullptr;
            _crumbs_sem->give();
            return run_lonely_mode();
        }

        lonely_mode = nullptr;

        _current_crumb = &_crumbs[_crumbs_head++];

        //re-use guided modes waypoint functionality
        ModeGuided::set_desired_location(*_current_crumb);

        //loop around the queue if we hit the end
        if (_crumbs_head >= crumbs_max) {
            _crumbs_head = 0;
        }

        _crumbs_sem->give();
    }

}




void ModeBreadcrumb::mavlink_packet_received(const mavlink_message_t &msg)
{


    if (rover.control_mode != &rover.mode_breadcrumb) {
        return;
    }
    if (msg.sysid != target_srcid) {
        return;
    }
    if (msg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        return;
    }

    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);

    if (!_crumbs_sem->take_nonblocking()) {
        return;
    }

    uint16_t crumbs_queue_size = (_crumbs_tail >= _crumbs_head) ? (_crumbs_tail - _crumbs_head) : crumbs_max - (_crumbs_head-_crumbs_tail);

    if (crumbs_queue_size >= crumbs_max) {
        _crumbs_sem->give();
        return;
    }

    //loop back around the queue to the beggining
    if (_crumbs_tail >= crumbs_max) {
        _crumbs_tail = 0;
    }

    _crumbs[_crumbs_tail].lat = packet.lat;
    _crumbs[_crumbs_tail].lng = packet.lon;
    _crumbs[_crumbs_tail++].alt = 0.0f; //rover doesn't use alt

    //target location and velocity used to dynamically change rover speed
    target_loc.lat = packet.lat;
    target_loc.lng = packet.lon;
    target_loc.alt = 0.0f;
    target_vel.x = packet.vx/100.0f; // cm/s to m/s
    target_vel.y = packet.vy/100.0f; // cm/s to m/s
    target_vel.z = 0.0f;

    _crumbs_sem->give();

}
