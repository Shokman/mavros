/**
 * @brief HIL plugin
 * @file hil.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros_msgs/HILSensor.h>
#include <mavros_msgs/HILControls.h>

namespace mavplugin {
/**
 * @brief HIL Sensor plugin
 */
class HILPlugin : public MavRosPlugin {
public:
	HILPlugin() :
		hil_nh("~hil"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		hil_controls_pub = hil_nh.advertise<mavros_msgs::HILControls>("controls", 10);
		hil_sensor_sub = hil_nh.subscribe("sensor", 10, &HILPlugin::hil_sensor_cb, this);

		uas->sig_connection_changed.connect(boost::bind(&HILPlugin::connection_cb, this, _1));
	};


        const message_map get_rx_handlers() {
                return {
                	MESSAGE_HANDLER(MAVLINK_MSG_ID_HIL_CONTROLS, &HILPlugin::handle_hil_controls)
                };
        }

private:
	std::recursive_mutex mutex;
	ros::NodeHandle hil_nh;
	UAS *uas;

	ros::Subscriber hil_sensor_sub;
	ros::Publisher hil_controls_pub;

	/* -*- low-level send functions -*- */

	void hil_sensor_send(const mavros_msgs::HILSensor::ConstPtr &hil_sensor) {
		mavlink_message_t msg;

		ros::Time time = ros::Time::now();

		mavlink_msg_hil_sensor_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time.toNSec()/1000,
				hil_sensor->xacc,
				hil_sensor->yacc,
				hil_sensor->zacc,
				hil_sensor->xgyro,
				hil_sensor->ygyro,
				hil_sensor->zgyro,
				hil_sensor->xmag,
				hil_sensor->ymag,
				hil_sensor->zmag,
				hil_sensor->abs_pressure,
				hil_sensor->diff_pressure,
				hil_sensor->pressure_alt,
				hil_sensor->temperature,
                                hil_sensor->fields_updated
				);
		UAS_FCU(uas)->send_message(&msg);
	}

        void handle_hil_controls(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {

                mavlink_hil_controls_t hil_controls_values;
                mavlink_msg_hil_controls_decode(msg, &hil_controls_values);

		auto hil_controls = boost::make_shared<mavros_msgs::HILControls>();

                std_msgs::Header header;
                header.stamp = uas->synchronise_stamp(hil_controls_values.time_usec);

		hil_controls->roll_ailerons = hil_controls_values.roll_ailerons;
		hil_controls->pitch_elevator = hil_controls_values.roll_ailerons;
		hil_controls->yaw_rudder = hil_controls_values.yaw_rudder;
		hil_controls->throttle = hil_controls_values.throttle;
		hil_controls->aux1 = hil_controls_values.aux1;
		hil_controls->aux2 = hil_controls_values.aux2;
		hil_controls->aux3 = hil_controls_values.aux3;
		hil_controls->aux4 = hil_controls_values.aux4;
		hil_controls->mode = hil_controls_values.mode;
		hil_controls->nav_mode = hil_controls_values.nav_mode;

		hil_controls_pub.publish(hil_controls);
	}

	/* -*- callbacks -*- */

	void connection_cb(bool connected) {
		lock_guard lock(mutex);
	}

	void hil_sensor_cb(const mavros_msgs::HILSensor::ConstPtr &req) {
//		if (!uas->is_ardupilotmega())

		hil_sensor_send(req);
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::HILPlugin, mavplugin::MavRosPlugin)

