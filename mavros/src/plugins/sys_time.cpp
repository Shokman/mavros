/**
 * @brief System Time plugin
 * @file sys_time.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2013 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <mavros/sys_time.h>

namespace mavplugin {

/**
 * Time syncronization status publisher
 *
 * Based on diagnistic_updater::FrequencyStatus
 */
	TimeSyncStatus::TimeSyncStatus(const std::string name, size_t win_size) :
		diagnostic_updater::DiagnosticTask(name),
		window_size_(win_size),
		min_freq_(0.01),
		max_freq_(10),
		tolerance_(0.1),
		times_(win_size),
		seq_nums_(win_size),
		last_dt(0),
		dt_sum(0)
	{
		clear();
	}

	void TimeSyncStatus::clear() {
		lock_guard lock(mutex);
		ros::Time curtime = ros::Time::now();
		count_ = 0;
		dt_sum = 0;

		for (int i = 0; i < window_size_; i++)
		{
			times_[i] = curtime;
			seq_nums_[i] = count_;
		}

		hist_indx_ = 0;
	}

	void TimeSyncStatus::tick(int64_t dt, uint64_t timestamp_us) {
		lock_guard lock(mutex);
		count_++;
		last_dt = dt;
		dt_sum += dt;
		last_ts = timestamp_us;
	}

	void TimeSyncStatus::set_timestamp(uint64_t timestamp_us) {
		lock_guard lock(mutex);
		last_ts = timestamp_us;
	}

	void TimeSyncStatus::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
		lock_guard lock(mutex);
		ros::Time curtime = ros::Time::now();
		int curseq = count_;
		int events = curseq - seq_nums_[hist_indx_];
		double window = (curtime - times_[hist_indx_]).toSec();
		double freq = events / window;
		seq_nums_[hist_indx_] = curseq;
		times_[hist_indx_] = curtime;
		hist_indx_ = (hist_indx_ + 1) % window_size_;

		if (events == 0) {
			stat.summary(2, "No events recorded.");
		}
		else if (freq < min_freq_ * (1 - tolerance_)) {
			stat.summary(1, "Frequency too low.");
		}
		else if (freq > max_freq_ * (1 + tolerance_)) {
			stat.summary(1, "Frequency too high.");
		}
		else {
			stat.summary(0, "Normal");
		}

		stat.addf("Events in window", "%d", events);
		stat.addf("Events since startup", "%d", count_);
		stat.addf("Duration of window (s)", "%f", window);
		stat.addf("Actual frequency (Hz)", "%f", freq);
		stat.addf("Last dt (ms)", "%0.3f", last_dt / 1000.0);
		stat.addf("Mean dt (ms)", "%0.3f", (count_)? dt_sum / count_ / 1000.0 : 0.0);
		stat.addf("Last system time (s)", "%0.6f", last_ts / 1e6);
	}



	SystemTimePlugin::SystemTimePlugin():
		uas(nullptr),
		dt_diag("Time Sync", 10),
		time_offset_us(0)
	{};

	void SystemTimePlugin::initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		double conn_system_time_d;

		uas = &uas_;

		nh.param("conn_system_time", conn_system_time_d, 0.0);
		nh.param<std::string>("frame_id", frame_id, "fcu");
		nh.param<std::string>("time_ref_source", time_ref_source, frame_id);

		diag_updater.add(dt_diag);

		time_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time_reference", 10);
		time_offset_pub = nh.advertise<std_msgs::Duration>("time_offset", 10);

		// timer for sending time sync messages
		if (conn_system_time_d > 0.0) {
			sys_time_timer = nh.createTimer(ros::Duration(conn_system_time_d),
					&SystemTimePlugin::sys_time_cb, this);
			sys_time_timer.start();
		}
	}


	std::string const SystemTimePlugin::get_name() const {
		return "SystemTime";
	}

	const MavRosPlugin::message_map SystemTimePlugin::get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_SYSTEM_TIME, &SystemTimePlugin::handle_system_time),
		};
	}

	void SystemTimePlugin::handle_system_time(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_system_time_t mtime;
		mavlink_msg_system_time_decode(msg, &mtime);

		uint64_t now_ms = ros::Time::now().toNSec() / 1000000;

		// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
		const bool fcu_time_valid = mtime.time_unix_usec > 1234567890ULL * 1000000;
		const bool ros_time_valid = now_ms > 1234567890ULL * 1000;

		int64_t offset_us = (now_ms - mtime.time_boot_ms) * 1000;
		int64_t dt = offset_us - time_offset_us;
		if (std::abs(dt) > 2000000 /* microseconds */) {
			ROS_WARN_THROTTLE_NAMED(10, "time", "TM: Large clock skew detected (%0.6f s). "
					"Resyncing clocks.", dt / 1e6);
			time_offset_us = offset_us;
			dt_diag.clear();
			dt_diag.set_timestamp(mtime.time_unix_usec);
		}
		else {
			time_offset_us = (time_offset_us + offset_us) / 2;
			dt_diag.tick(dt, mtime.time_unix_usec);
		}

		if (fcu_time_valid) {
			// continious publish for ntpd
			sensor_msgs::TimeReferencePtr time_unix = boost::make_shared<sensor_msgs::TimeReference>();
			ros::Time time_ref(
					mtime.time_unix_usec / 1000000,			// t_sec
					(mtime.time_unix_usec % 1000000) * 1000);	// t_nsec

			time_unix->source = time_ref_source;
			time_unix->time_ref = time_ref;
			time_unix->header.stamp = ros::Time::now();

			time_ref_pub.publish(time_unix);
		}
		else {
			ROS_WARN_THROTTLE_NAMED(60, "time", "TM: Wrong GPS time.");
		}

		// offset publisher
		std_msgs::DurationPtr offset = boost::make_shared<std_msgs::Duration>();
		ros::Duration time_ref(
				time_offset_us / 1000000,		// t_sec
				(time_offset_us % 1000000) * 1000);	// t_nsec

		offset->data = time_ref;

		uas->set_time_offset(time_offset_us);
		time_offset_pub.publish(offset);
	}

	void SystemTimePlugin::sys_time_cb(const ros::TimerEvent &event) {
		mavlink_message_t msg;

		uint64_t time_unix_usec = ros::Time::now().toNSec() / 1000;  // nano -> micro

		mavlink_msg_system_time_pack_chan(UAS_PACK_CHAN(uas), &msg,
				time_unix_usec,
				0
				);
		UAS_FCU(uas)->send_message(&msg);
	}

}; // namespace mavplugin

