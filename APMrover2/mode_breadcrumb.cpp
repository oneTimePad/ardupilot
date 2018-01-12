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

    last_crumb_received = rover.current_loc;

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

        //maintain at greater than distance_to_stop from target
        if (fabsf(distance_to_vehicle) > distance_to_stop) {
            //we need to speed up
            to_vehicle *= closure_speed * 100; // m/s to cm/s (which set_desired_speed takes)
            //hal.console->printf("%s\n","go faster");
        } else if (fabsf(distance_to_vehicle) <= distance_to_stop) {
            //too close. back up
            hal.console->printf("%s\n","back up");
            to_vehicle = -to_vehicle;
            return;
        }

        to_vehicle += target_vel;

        float desired_speed = to_vehicle.length();
        //hal.console->printf("%f\n",desired_speed);
        Mode::set_desired_speed(desired_speed);

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

    //target location and velocity used to dynamically change rover speed
    target_loc.lat = packet.lat;
    target_loc.lng = packet.lon;
    target_loc.alt = 0.0f;
    target_vel.x = packet.vx/100.0f; // cm/s to m/s
    target_vel.y = packet.vy/100.0f; // cm/s to m/s
    target_vel.z = 0.0f;

    uint16_t crumbs_queue_size = (_crumbs_tail >= _crumbs_head) ? (_crumbs_tail - _crumbs_head) : crumbs_max - (_crumbs_head-_crumbs_tail);

    if (crumbs_queue_size >= crumbs_max) {
        _crumbs_sem->give();
        return;
    }

    Location new_crumb;
    new_crumb.lat = packet.lat;
    new_crumb.lng = packet.lon;
    new_crumb.alt = 0.0f;//packet.relative_alt / 10; // mm -> cm


    //hal.console->printf("crumb size %d\n",crumbs_queue_size);
    Vector3f to_vehicle = location_3d_diff_NED(last_crumb_received, new_crumb);

    const float distance_to_last_crum = to_vehicle.length();
    //hal.console->printf("%f\n",distance_to_last_crum);
    bool add_crumb = distance_to_last_crum > distance_to_stop;

    //hal.console->printf("%lu %lu %lu %lu %f %f\n",new_crumb.lat,last_crumb_added.lat,new_crumb.lng,last_crumb_added.lng,
    //                                                new_crumb.alt,last_crumb_added.alt);
    //hal.console->printf("%d %f\n",add_crumb,distance_to_last_crum);

    //loop back around the queue to the beggining
    if (_crumbs_tail >= crumbs_max) {
        _crumbs_tail = 0;
    }

    if (add_crumb){
        hal.console->printf("ADDED crumb\n");
        last_crumb_received = new_crumb;
        _crumbs[_crumbs_tail++] = new_crumb;
    }

    _crumbs_sem->give();

}
