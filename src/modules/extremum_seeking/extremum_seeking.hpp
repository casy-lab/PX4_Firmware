/*
 * @author Lorenzo Gentilini (University of Bologna)
 * May, 2020
 */

#pragma once

#include <px4_module.h>
#include <px4_module_params.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_arva.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/freq_control.h>
#include <uORB/topics/setpoint_general.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/uORB.h>

#include "px4_custom_mode.h"

#include <math.h>
#include <matrix/matrix/math.hpp>

#define Z_REF 6.1268
#define EPSILON 0.35
#define FREQ 10

/** Extremum Seeking Parameters **/
// For 17Hz
#define A 0.9972
#define B 0.0028
#define C 0.9986
#define D -0.0036
#define E -0.0018

// TF: 1/50s+1
#define FA 0.9989
#define FB 0.0312
#define FC 0.0358
#define FD 5.5969e-04

#define ALPHA 2
#define OMEGA 0.65

/** Bounded Update Rate ES Parameters **/
#define FA_BUR 0.9950
#define FB_BUR 0.0625
#define FC_BUR 0.0796
#define FD_BUR 0.0025

#define OMEGA_BUR 0.65
#define ALPHA_BUR 10
#define KAPPA_BUR 0.05

#define FA_ARVA -0.2381
#define FB_ARVA 0.5
#define FC_ARVA 0.9433
#define FD_ARVA 0.6190

typedef enum{TAKEOFF, SEARCH} uav_state;

extern "C" __EXPORT int extremum_seeking_main(int argc, char *argv[]);

/** Extremum Seeking **/
class ExSeeking{
	private:
	double zPlus, z, yZ;
	double fPlus, f, alpha;
	double uX, uXPlus, uY, uYPlus;
	double time;

	public:
	// Constructor
	ExSeeking();

	// Destructor
	~ExSeeking();

	// Main Function
	matrix::Matrix<double, 2, 1> update(double y);

	// Reset Function
	void reset();
};


/** Bounded Update Rate ES **/
class ExSeekingBUR{
	private:
	double fPlus, f, alpha, xOld, yOld;
	double time, previousTime;

	public:
	// Constructor
	ExSeekingBUR();

	// Destructor
	~ExSeekingBUR();

	// Main Function
	matrix::Matrix<double, 3, 1> update(double y);

	// Reset Function
	void reset();
};

class ESModule : public ModuleBase<ESModule>, public ModuleParams{
	public:
		// Constructor
		ESModule();

		// Destructor
		~ESModule();

		// Module Base Fuctions
		static int task_spawn(int argc, char *argv[]);
		static ESModule *instantiate(int argc, char *argv[]);
		static int custom_command(int argc, char *argv[]);
		static int print_usage(const char *reason = nullptr);
		void run() override;
		int print_status() override;

	private:
		// Attributes
		ExSeekingBUR* _ref_gen;
		uav_state _state = TAKEOFF;
		double fPlus = 0.0, f = 0.0;
		bool isFirstTime = true;
		double _arva_filtered = 0.0;
		int count = 0;

		matrix::Matrix<double, 3, 3> iRp;
		matrix::Matrix<double, 3, 1> Op;

		hrt_abstime actualTime{hrt_absolute_time()};
		hrt_abstime previousTime{hrt_absolute_time()};

		// Structures
		struct vehicle_local_position_s _local_pos;
		struct sensor_arva_s			_sens_arva;
		struct setpoint_general_s		_sp_triplet;
		struct freq_control_s			_freq;
		struct vehicle_control_mode_s	_mode;
		struct distance_sensor_s		_ground_dist;

		// Subscriptions
		uORB::Subscription						 _parameter_update_sub{ORB_ID(parameter_update)};
		uORB::Subscription						 _arva_sub{ORB_ID(sensor_arva)};
		uORB::Subscription       				 _local_pos_sub{ORB_ID(vehicle_local_position)};
		uORB::Subscription						 _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
		uORB::Subscription						 _distance_sensor_sub{ORB_ID(distance_sensor)};

		// Publications
		uORB::Publication<setpoint_general_s> 	_sp_triplet_pub{ORB_ID(setpoint_general)};
		uORB::Publication<freq_control_s>		_freq_pub{ORB_ID(freq_control)};
};

