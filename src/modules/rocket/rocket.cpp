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
#include <queue>

#include <px4_tasks.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/rocket.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>

extern "C" __EXPORT int rocket_main(int argc, char *argv[]);

int rocket_thread_main(void);
constexpr float PI = (float)M_PI;

bool suppress_ekf_gps = false;

static float acceleration = 0.0;
static float accel_vertical = 0.0;
static constexpr float MASS = 0.625; // kilograms
static constexpr float AIR_DENSITY = 1.225; // kg / m^3

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
        _state(TESTING),
        _armed(false),
        _target_altitude(target_altitude),
        _deployment_altitude(deployment_altitude),
        _trigger_deployment_velocity(35.0),
        _testing_angle(0.0),
        _current_angle(0.0),
        _cda_testing_angle(0.0),
        _pid(KP, KI, KD),
        _counter(0)
    {}

    typedef enum {
        TESTING,
        PRELAUNCH,
        BOOST,
        ASCENT,
        DESCENT,
        RECOVERY,
        EMERGENCY_RECOVERY
    } FlightState;

    float estimate_apogee(float altitude, float velocity)
    {
        if (!isnan(altitude) && (!isnan(velocity))) {
            float term_vel_sqrd = (2 * MASS * GRAVITY) / (estimate_cda(_current_angle) * AIR_DENSITY); // terminal velocity squared
            return altitude + ((term_vel_sqrd / (2 * GRAVITY)) * logf((powf(velocity, 2) + term_vel_sqrd) / term_vel_sqrd));
        } else {
            PX4_WARN("One or more sensors returned NaN");
            return NAN;
        }
    }

    float update_brake_angle(float altitude, float velocity) {
        float apogee_alt = estimate_apogee(altitude, velocity);
        if (!isnan(apogee_alt)) {
            _error = apogee_alt - _target_altitude;
            _current_angle += _pid.update(_error);
            if (_current_angle > (PI/2)) {
                _current_angle = (PI/2);
            }
            else if (_current_angle < 0) {
                _current_angle = 0;
            }
        }
        return _current_angle;
    }

    FlightState update_state(float altitude, float velocity) {
        if (!isnan(altitude) and !isnan(velocity)) {
            switch(_state) {
                case TESTING:
                    if (_armed) {
                        _testing_angle += 0.05f;
                        if (_testing_angle > (PI/2)) {
                            _testing_angle = 0.0;
                        }
                        if (time_since_armed() > 5000000) {
                            _testing_angle = 0.0;
                        }
                        // give time for drag brakes to get back to zero degrees
                        if (time_since_armed() > 5500000) {
                            _state = PRELAUNCH;
                        }
                    }
                    break;
                case PRELAUNCH:
                    if (velocity > 5) {
                        _counter++;
                    } else {
                        _counter = 0;
                    }

                    if (_counter > 3) {
                        _state = BOOST;
                        _counter = 0;
                    }
                    break;
                case BOOST:
                    if (accel_vertical > 0) {
                        _counter++;
                    } else {
                        _counter = 0;
                    }

                    if (_counter > 3) {
                        _state = ASCENT;
                        _coast_time = hrt_absolute_time();
                        _counter = 0;
                    }
                    break;
                case ASCENT:
                    if ((hrt_absolute_time() - _coast_time) > 1000000) {
                        if ((hrt_absolute_time() - _coast_time) < 1700000) {
                            _cda_testing_angle = (ANGLES[0]*(PI/180));
                        } else if ((hrt_absolute_time() - _coast_time) < 2400000) {
                            _cda_testing_angle = (ANGLES[1]*(PI/180));
                        } else {
                            _cda_testing_angle = (ANGLES[2]*(PI/180));
                        }
                    }

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
                    if ((altitude < _deployment_altitude) || (-velocity > _trigger_deployment_velocity)) {
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
            suppress_ekf_gps = (_state == BOOST);
        }
        return _state;
    }

    void actuate(orb_advert_t actuators_0_pub) {
        struct actuator_controls_s actuators_out_0;

        actuators_out_0.timestamp = hrt_absolute_time();
        actuators_out_0.control[0] = angle_to_command(0.0f);
        actuators_out_0.control[1] = angle_to_command(0.0f);
        actuators_out_0.control[2] = 1.0;

        if(_state == TESTING) {
            actuators_out_0.control[0] = angle_to_command(_testing_angle);
            actuators_out_0.control[1] = angle_to_command(_testing_angle);
            actuators_out_0.control[2] = angle_to_command(_testing_angle);
        }

        if ((_state == PRELAUNCH) || (_state == RECOVERY)) {
            actuators_out_0.control[3] = -1.0;
        } else {
            actuators_out_0.control[3] = 1.0;
        }

        if (_state == ASCENT) {
            if (DRAG_BRAKES_ENABLED) {
                if (CDA_TESTING) {
                    actuators_out_0.control[0] = angle_to_command(_cda_testing_angle);
                    actuators_out_0.control[1] = angle_to_command(_cda_testing_angle);
                } else {
                    if ((hrt_absolute_time() - _coast_time) > 2250000) {
                        actuators_out_0.control[0] = angle_to_command(_current_angle);
                        actuators_out_0.control[1] = angle_to_command(_current_angle);
                    }
                }
            }
        }

        if ((_state == RECOVERY) || (_state == EMERGENCY_RECOVERY)) {
            actuators_out_0.control[2] = -1.0;
        }

        orb_publish(ORB_ID(actuator_controls_0), actuators_0_pub, &actuators_out_0);
    }

    FlightState _state;
    float _error;
    bool _armed;
    hrt_abstime _armed_time;


private:
    static constexpr float KP = 0.008;
    static constexpr float KI = 0.0;
    static constexpr float KD = 0.0;
    static constexpr float GRAVITY = 9.80665; // m/s^2
    static constexpr bool CDA_TESTING = false;
    static constexpr bool DRAG_BRAKES_ENABLED = true;
    const int ANGLES[3] = {30, 60, 90}; // degrees, used for CdA testing
    const float COEFS[4] = {-0.00318461, 0.0100433, 0.00851401, 0.00318692}; // used to calculate CdA from brake angle

    float _target_altitude;
    float _deployment_altitude;
    float _trigger_deployment_velocity;
    float _testing_angle;
    float _current_angle;
    float _cda_testing_angle;
    Pid _pid;
    int _counter;
    hrt_abstime _coast_time;

    float estimate_cda(float angle) {
        return (COEFS[3] + (COEFS[2]*angle) + (COEFS[1]*powf(angle, 2)) + (COEFS[0]*powf(angle, 3)));
    }

    hrt_abstime time_since_armed() {
        return (hrt_absolute_time() - _armed_time);
    }

    // Takes angle in radians and converts to servo command (-1 to 1)
    float angle_to_command(float angle) {
        return (((angle / (PI/2)) * 2) - 1);
    }

};


int rocket_thread_main(void)
{

    RocketController controller = RocketController(236.22, 195);
    int emergency_counter = 0;
    float prev_alt = 0.0;
    float base_alt = 0.0;
    std::queue<float> altitudes_buffer;
    hrt_abstime prev_timestamp = hrt_absolute_time();

    float altitude = 0.0;
    float velocity = 0.0;
    float speed = 0.0;

    float raw_velocity = 0.0;
    hrt_abstime raw_velocity_timestamp = hrt_absolute_time();
    float prev_velocity = 0.0;
    hrt_abstime prev_velocity_timestamp = hrt_absolute_time();

    /* subscribe to topics */
    int gps_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
    int baro_sub_fd = orb_subscribe(ORB_ID(sensor_baro)); // used for emergency parachute deployment
    int status_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    /* limit the update rate to 20 Hz */
    orb_set_interval(gps_sub_fd, 50);
    orb_set_interval(baro_sub_fd, 50);
    orb_set_interval(status_sub_fd, 50);
    orb_set_interval(sensor_sub_fd, 50);

    struct rocket_s rkt;
    memset(&rkt, 0, sizeof(rkt));
    orb_advert_t rkt_pub = orb_advertise(ORB_ID(rocket), &rkt);

    struct actuator_controls_s actuators_out_0;
    memset(&actuators_out_0, 0, sizeof(actuators_out_0));
    orb_advert_t actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuators_out_0);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[4];
    fds[0].fd = gps_sub_fd;
    fds[0].events = POLLIN;
    fds[1].fd = baro_sub_fd;
    fds[1].events = POLLIN;
    fds[2].fd = status_sub_fd;
    fds[2].events = POLLIN;
    fds[3].fd = sensor_sub_fd;
    fds[3].events = POLLIN;

    int error_counter = 0;

    while(true) {
        /* wait for sensor update of 4 file descriptors for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 4, 1000);

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
                struct vehicle_gps_position_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vehicle_gps_position), gps_sub_fd, &raw);

                prev_velocity = raw_velocity;
                prev_velocity_timestamp = raw_velocity_timestamp;
                raw_velocity = -raw.vel_d_m_s;
                raw_velocity_timestamp = raw.timestamp;
                speed = sqrtf(powf(raw.vel_d_m_s, 2) + powf(raw.vel_e_m_s, 2) + powf(raw.vel_n_m_s, 2));
            }

            if (fds[1].revents & POLLIN) {
                struct sensor_baro_s baro;
                orb_copy(ORB_ID(sensor_baro), baro_sub_fd, &baro);

                if (controller._state == RocketController::PRELAUNCH) {
                    if (altitudes_buffer.size() > 25) {
                        base_alt = altitudes_buffer.front();
                        altitudes_buffer.pop();
                    }
                    altitudes_buffer.push(baro.altitude);
                }

                altitude = baro.altitude-base_alt;
                velocity = ((raw_velocity-prev_velocity)/(raw_velocity_timestamp-prev_velocity_timestamp))*(hrt_absolute_time()-raw_velocity_timestamp) + raw_velocity;

                rkt.input_altitude = altitude;
                rkt.input_velocity = velocity;
                float estimated_cda = ((acceleration * MASS) / (powf(speed, 2) * AIR_DENSITY * 0.5f)) * 10000;
                rkt.estimated_cda = ((estimated_cda > 1000) ? 1000 : estimated_cda); // Cap CdA measurements at 1000
                rkt.apogee_estimate = controller.estimate_apogee(altitude, velocity);
                float brake_angle = controller.update_brake_angle(altitude, velocity);
                rkt.target_drag_brake_angle = brake_angle * (180/PI);
                rkt.error = controller._error;
                rkt.flight_state = controller.update_state(altitude, raw_velocity);
                rkt.emergency_counter = emergency_counter;
                rkt.timestamp = hrt_absolute_time();
                orb_publish(ORB_ID(rocket), rkt_pub, &rkt);


                // emergency parachute deployment
                if ((((prev_alt - baro.altitude) / ((baro.timestamp - prev_timestamp) / 1000000.0f)) > 45) && (controller._state != RocketController::RECOVERY)) {
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

            if (fds[2].revents & POLLIN) {
                struct vehicle_status_s status;
                orb_copy(ORB_ID(vehicle_status), status_sub_fd, &status);
                if (!controller._armed) {
                    controller._armed_time = hrt_absolute_time();
                }
                if (status.arming_state == 2) {
                    controller._armed = true;
                } else {
                    controller._armed = false;
                }
            }

            if (fds[3].revents & POLLIN) {
                struct sensor_combined_s sensors;
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &sensors);
                acceleration = sqrtf(powf(sensors.accelerometer_m_s2[0], 2) + powf(sensors.accelerometer_m_s2[1], 2) + powf(sensors.accelerometer_m_s2[2], 2));
                accel_vertical = sensors.accelerometer_m_s2[2];
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
