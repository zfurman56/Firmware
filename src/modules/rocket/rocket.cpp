/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rocket.cpp
 * Main file for rocket code
 *
 * @author Zach Furman <zach.furman1@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <poll.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/rocket.h>

extern "C" __EXPORT int rocket_main(int argc, char *argv[]);

class Pid {

public:
    Pid(float kp, float ki, float kd):
        _kp(kp),
        _ki(ki),
        _kd(kd),
        _previous_error(0.0),
        _integrated_error(0.0)
    {}

    float update(float error) {
        _integrated_error += error;
        float differential_error = (error - _previous_error);

        _previous_error = error;

        return (_kp*error) + (_ki*_integrated_error) + (_kd*differential_error);
    }

private:
    float _kp;  // Proportional constant
    float _ki;  // Integral constant
    float _kd;  // Differential constant
    float _previous_error;
    float _integrated_error;
};

class RocketController {
public:
    RocketController(float target_altitude):
        _target_altitude(target_altitude),
        _current_angle(0.0),
        _pid(KP, KI, KD)
    {}

    float estimate_apogee(float altitude, float velocity)
    {
        float forces;
        float sim_time = 0;

        while (true)
        {
            forces = (GRAVITY * MASS);
            forces += drag_force(_current_angle, velocity);

            velocity += ((forces/MASS) * STEP_SIZE);
            altitude += (velocity * STEP_SIZE);

            sim_time += STEP_SIZE;

            if (velocity < 0) break;
        }

        return altitude;
    }

    float update_brake_angle(float altitude, float velocity) {
        _error = estimate_apogee(altitude, velocity) - _target_altitude;
        _current_angle += _pid.update(_error);
        if (_current_angle > (M_PI/2)) {
            _current_angle = (M_PI/2);
        }
        else if (_current_angle < 0) {
            _current_angle = 0;
        }
        return _current_angle;
    }

    float _error;


private:
    static constexpr float KP = 0.008;
    static constexpr float KI = 0.0;
    static constexpr float KD = 0.0;
    static constexpr float GRAVITY = -9.8; // m/s^2
    static constexpr float MASS = 0.625; // kilograms
    static constexpr float DRAG_FACTOR = 0.0011;
    static constexpr float DRAG_GAIN = 7.0;
    static constexpr float STEP_SIZE = 0.01; // seconds

    float _target_altitude;
    float _current_angle;
    Pid _pid;

    float drag_force(float drag_brake_angle, float velocity) {
        return DRAG_FACTOR * (1 + (DRAG_GAIN * pow(sin(drag_brake_angle), 2))) * -pow(velocity, 2);
    }
};

int rocket_main(int argc, char *argv[])
{

    RocketController controller = RocketController(236.22);

    /* subscribe to vehicle_local_position topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_sub_fd, 200);

    struct rocket_s rkt;
    memset(&rkt, 0, sizeof(rkt));
    orb_advert_t rkt_pub = orb_advertise(ORB_ID(rocket), &rkt);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    int error_counter = 0;

    for (int i = 0; i < 80; i++) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct vehicle_local_position_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vehicle_local_position), sensor_sub_fd, &raw);
                PX4_INFO("Estimated apogee: %8.4f", controller.estimate_apogee(-raw.z, -raw.vz));
                PX4_INFO("Current altitude: %8.4f", -raw.z);

                rkt.input_altitude = -raw.z;
                rkt.input_velocity = -raw.vz;
                rkt.apogee_estimate = controller.estimate_apogee(-raw.z, -raw.vz);
                rkt.target_drag_brake_angle = controller.update_brake_angle(-raw.z, -raw.vz) * (180/M_PI);
                rkt.error = controller._error;
                rkt.timestamp = hrt_absolute_time();
                orb_publish(ORB_ID(rocket), rkt_pub, &rkt);

            }


            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
        }
    }

    PX4_INFO("shutting down...");

    return 0;
}