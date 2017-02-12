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

#include <px4_tasks.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/rocket.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

extern "C" __EXPORT int rocket_main(int argc, char *argv[]);

int rocket_thread_main(void);
constexpr float PI = (float)M_PI;

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
    RocketController(float target_altitude, float deployment_altitude):
        _state(PRELAUNCH),
        _target_altitude(target_altitude),
        _deployment_altitude(deployment_altitude),
        _current_angle(0.0),
        _pid(KP, KI, KD),
        _counter(0)
    {}

    typedef enum {
        PRELAUNCH,
        ASCENT,
        DESCENT,
        RECOVERY,
        EMERGENCY_RECOVERY
    } FlightState;

    float estimate_apogee(float altitude, float velocity)
    {
        if (!isnan(altitude) && (!isnan(velocity))) {
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
        } else {
            PX4_WARN("One or more sensors returned NaN");
            return NAN;
        }
    }

    float update_brake_angle(float altitude, float velocity) {
        float apogee_alt = estimate_apogee(altitude, velocity);
        if (!isnan(apogee_alt)) {
            if (_state == ASCENT) {
                _error = apogee_alt - _target_altitude;
                _current_angle += _pid.update(_error);
                if (_current_angle > (PI/2)) {
                    _current_angle = (PI/2);
                }
                else if (_current_angle < 0) {
                    _current_angle = 0;
                }
            } else if (_state == DESCENT) {
                _current_angle = 0.5f;
            }
        }
        return _current_angle;
    }

    FlightState update_state(float altitude, float velocity) {
        if (!isnan(altitude) and !isnan(velocity)) {
            switch(_state) {
                case PRELAUNCH:
                    if (velocity > 5) {
                        _counter++;
                    } else {
                        _counter = 0;
                    }

                    if (_counter > 3) {
                        _state = ASCENT;
                        _counter = 0;
                    }
                    break;
                case ASCENT:
                    if (velocity < 0) {
                        _counter++;
                    } else {
                        _counter = 0;
                    }

                    if (_counter > 3) {
                        _state = DESCENT;
                        _counter = 0;
                    }
                    break;
                case DESCENT:
                    if (altitude < _deployment_altitude) {
                        _counter++;
                    }

                    if (_counter > 4) {
                        _state = RECOVERY;
                        _counter = 0;
                    }
                    break;
                case RECOVERY:
                    break;
                case EMERGENCY_RECOVERY:
                    break;
            }
        }
        return _state;
    }

    void actuate(orb_advert_t actuators_0_pub) {
        struct actuator_controls_s actuators_out_0;

        actuators_out_0.timestamp = hrt_absolute_time();
        actuators_out_0.control[0] = angle_to_command(0.0f);
        actuators_out_0.control[1] = angle_to_command(0.0f);
        actuators_out_0.control[2] = -1.0;

        if (_state == PRELAUNCH) {
            actuators_out_0.control[3] = -1.0;
        } else {
            actuators_out_0.control[3] = 1.0;
        }

        if (_state == ASCENT) {
            actuators_out_0.control[0] = angle_to_command(_current_angle);
            actuators_out_0.control[1] = angle_to_command(_current_angle);
        }

        if (_state == DESCENT) {
            actuators_out_0.control[0] = angle_to_command(1.0f);
            actuators_out_0.control[1] = angle_to_command(1.0f);
        }

        if ((_state == RECOVERY) || (_state == EMERGENCY_RECOVERY)) {
            actuators_out_0.control[0] = angle_to_command(1.0f);
            actuators_out_0.control[1] = angle_to_command(1.0f);
            actuators_out_0.control[2] = 1.0;
        }

        orb_publish(ORB_ID(actuator_controls_0), actuators_0_pub, &actuators_out_0);
    }

    FlightState _state;
    float _error;


private:
    static constexpr float KP = 0.008;
    static constexpr float KI = 0.0;
    static constexpr float KD = 0.0;
    static constexpr float GRAVITY = -9.80665; // m/s^2
    static constexpr float MASS = 0.625; // kilograms
    static constexpr float DRAG_FACTOR = 0.0011;
    static constexpr float DRAG_GAIN = 7.0;
    static constexpr float STEP_SIZE = 0.01; // seconds

    float _target_altitude;
    float _deployment_altitude;
    float _current_angle;
    Pid _pid;
    int _counter;

    float drag_force(float drag_brake_angle, float velocity) {
        return DRAG_FACTOR * (1 + (DRAG_GAIN * powf(sin(drag_brake_angle), 2))) * -powf(velocity, 2);
    }

};

// Takes angle in radians and converts to servo command (-1 to 1)
float angle_to_command(float angle) {
    return (((angle / (PI/2)) * 2) - 1);
}

int rocket_thread_main(void)
{

    RocketController controller = RocketController(236.22, 200);
    int emergency_counter = 0;
    float prev_alt = 0.0;
    float base_altitude = -2000.0; // used as a dummy value to tell the program to initialize it
    hrt_abstime prev_timestamp = hrt_absolute_time();

    /* subscribe to vehicle_local_position topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
    int baro_sub_fd = orb_subscribe(ORB_ID(sensor_baro)); // used for emergency parachute deployment
    /* limit the update rate to 20 Hz */
    orb_set_interval(sensor_sub_fd, 50);
    orb_set_interval(baro_sub_fd, 50);

    struct rocket_s rkt;
    memset(&rkt, 0, sizeof(rkt));
    orb_advert_t rkt_pub = orb_advertise(ORB_ID(rocket), &rkt);

    struct actuator_controls_s actuators_out_0;
    memset(&actuators_out_0, 0, sizeof(actuators_out_0));
    orb_advert_t actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuators_out_0);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[2];
    fds[0].fd = sensor_sub_fd;
    fds[0].events = POLLIN;
    fds[1].fd = baro_sub_fd;
    fds[1].events = POLLIN;

    int error_counter = 0;

    while(true) {
        /* wait for sensor update of 2 file descriptors for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 2, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_WARN("Got no data within a second");

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

                rkt.input_altitude = -raw.z;
                rkt.input_velocity = -raw.vz;
                rkt.apogee_estimate = controller.estimate_apogee(-raw.z, -raw.vz);
                float brake_angle = controller.update_brake_angle(-raw.z, -raw.vz);
                rkt.target_drag_brake_angle = brake_angle * (180/PI);
                rkt.error = controller._error;
                rkt.flight_state = controller.update_state(-raw.z, -raw.vz);
                rkt.emergency_counter = emergency_counter;
                rkt.timestamp = hrt_absolute_time();
                orb_publish(ORB_ID(rocket), rkt_pub, &rkt);

            }

            if (fds[1].revents & POLLIN) {
                struct sensor_baro_s baro;
                orb_copy(ORB_ID(sensor_baro), baro_sub_fd, &baro);

                // Set the base altitude if it's not set yet
                if (base_altitude < -1000) {
                    base_altitude = baro.altitude;
                }

                if ((((prev_alt - baro.altitude) / ((baro.timestamp - prev_timestamp) / 1000000)) > 5) && (baro.altitude < (180 + base_altitude)) && (controller._state != RocketController::RECOVERY)) {
                    emergency_counter++;
                } else {
                    emergency_counter = 0;
                }

                if (emergency_counter > 4) {
                    PX4_ERR("Emergency recovery triggered");
                    controller._state = RocketController::EMERGENCY_RECOVERY;
                }

                prev_alt = baro.altitude;
                prev_timestamp = baro.timestamp;
            }

            controller.actuate(actuators_0_pub);

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
        }
    }

    PX4_INFO("shutting down...");

    return 0;
}

int rocket_main(int argc, char *argv[]) {
    int task = -1;

    task = px4_task_spawn_cmd("rocket",
                   SCHED_DEFAULT,
                   SCHED_PRIORITY_DEFAULT,
                   8192,
                   (px4_main_t)rocket_thread_main,
                   nullptr);

    if (task < 0) {
        PX4_ERR("Task start failed: %d", errno);
        return -errno;
    }

    return OK;
}
