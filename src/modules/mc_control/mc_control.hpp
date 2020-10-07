/**
 * @file mc_control.hpp
 * Multicopter Controller Header File.
 */

// Includes
#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <vtol_att_control/vtol_type.h>
#include <conversion/rotation.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>
#include <float.h>

#include <lib/controllib/blocks.hpp>
#include <lib/FlightTasks/FlightTasks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/WeatherVane/WeatherVane.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/mixer/mixer.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_posix.h>
#include <px4_tasks.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationQueued.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/tracking_errors.h>
#include <uORB/topics/freq_control.h>

#include "Utility/ControlMath.hpp"
#include "Takeoff.hpp"
#include "IMBControl/IMBControl.hpp"

typedef struct{
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
}PositionControlStates;

using namespace time_literals;

/**
 * Multicopter control app start / stop handling function
 */
extern "C" __EXPORT int mc_control_main(int argc, char *argv[]);

class MulticopterControl : 
	public ModuleBase<MulticopterControl>, public control::SuperBlock, public ModuleParams, public px4::WorkItem{

public:
	// Constructor
	MulticopterControl();

	// Destructor
	virtual ~MulticopterControl() override;

	// ModuleBase Functions
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	void Run() override;
	bool init();
	int print_status() override;

private:
	// Objects
	Takeoff _takeoff; 						// Takeoff State Machine
	control::BlockDerivative _vel_x_deriv; 	// Velocity derivative in X
	control::BlockDerivative _vel_y_deriv; 	// Velocity derivative in Y
	control::BlockDerivative _vel_z_deriv;	// Velocity derivative in Z
	FlightTasks _flight_tasks;	 			// Class for setpoint task-dependent
	PositionControlStates _states; 		    // Info for Position Control
	IMBControl _IMBControl;					// Internal Model Based Control

	MultirotorMixer::saturation_status _saturation_status{};
	systemlib::Hysteresis _failsafe_land_hysteresis{false};
	WeatherVane *_wv_controller{nullptr};
	
	// ******************************************************************************* //

	// Structs
	struct vehicle_status_s 			_vehicle_status{};
	struct vehicle_attitude_setpoint_s	_att_sp{};
	struct vehicle_control_mode_s		_control_mode{};
	struct vehicle_local_position_s 	_local_pos{};
	struct home_position_s				_home_pos{};
	struct landing_gear_s 				_landing_gear{};
    struct actuator_controls_s			_actuators{};
    struct battery_status_s				_battery_status{};
	struct vehicle_angular_velocity_s 	angular_velocity{};
    struct vehicle_attitude_s 			att{};
	struct tracking_errors_s			_errors{};
	struct freq_control_s				_freq{};

	struct vehicle_land_detected_s _vehicle_land_detected = {
		.timestamp = 0,
		.alt_max = -1.0f,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};
	// ******************************************************************************* //

	// Variables
	static constexpr const float initial_update_rate_hz = 250.f;

	hrt_abstime	_time_stamp_last_loop{0};
	hrt_abstime _last_warn{0};
	hrt_abstime _task_start{hrt_absolute_time()};

	hrt_abstime actualTime{hrt_absolute_time()};
	hrt_abstime previousTime{hrt_absolute_time()};

	bool _in_failsafe{false};
	bool _actuators_0_circuit_breaker_enabled{false};

	float _thrust_sp{0.0f};
	float _thrust_sp1{0.0f};
	float _loop_update_rate_hz{initial_update_rate_hz};
	float _dt_accumulator{0.0f};
	float _man_yaw_sp{0.f};

	int _task_failure_count{0};
	int _loop_counter{0};
	int8_t _old_landing_gear_position{landing_gear_s::GEAR_KEEP};

    matrix::Vector3f _att_control;
	matrix::Vector3f _att_control1;
	matrix::Vector3f _rates_sp;
	// ******************************************************************************* //

	// Constants
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) 	_param_mpc_tko_ramp_t,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) 	_param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_VEL_MANUAL>) 	_param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) 	_param_mpc_xy_cruise,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) 	_param_mpc_land_speed,
		(ParamFloat<px4::params::MPC_TKO_SPEED>) 	_param_mpc_tko_speed,
		(ParamFloat<px4::params::MPC_LAND_ALT2>) 	_param_mpc_land_alt2,
		(ParamInt<px4::params::MPC_POS_MODE>) 		_param_mpc_pos_mode,
		(ParamInt<px4::params::MPC_AUTO_MODE>) 		_param_mpc_auto_mode,
		(ParamInt<px4::params::MPC_ALT_MODE>) 		_param_mpc_alt_mode,
		(ParamFloat<px4::params::MPC_SPOOLUP_TIME>)	_param_mpc_spoolup_time,
		(ParamFloat<px4::params::MPC_TILTMAX_LND>) 	_param_mpc_tiltmax_lnd,
		(ParamFloat<px4::params::MPC_THR_MIN>)		_param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_HOVER>)	_param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MAX>)		_param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_Z_VEL_P>)		_param_mpc_z_vel_p,
		(ParamFloat<px4::params::MC_ROLL_P>) 		_param_mc_roll_p,
		(ParamFloat<px4::params::MC_ROLLRATE_P>) 	_param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) 	_param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) 	_param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) 	_param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) 	_param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) 	_param_mc_rollrate_k,
		(ParamFloat<px4::params::MC_PITCH_P>) 		_param_mc_pitch_p,
		(ParamFloat<px4::params::MC_PITCHRATE_P>) 	_param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) 	_param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) 	_param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) 	_param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) 	_param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) 	_param_mc_pitchrate_k,
		(ParamFloat<px4::params::MC_YAW_P>) 		_param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAWRATE_P>) 	_param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) 	_param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) 	_param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) 	_param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) 	_param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) 	_param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_DTERM_CUTOFF>) 	_param_mc_dterm_cutoff,
		(ParamFloat<px4::params::MC_ROLLRATE_MAX>) 	_param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>) 	_param_mc_yawrate_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) 	_param_mpc_man_y_max,
		(ParamFloat<px4::params::MC_ACRO_R_MAX>) 	_param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) 	_param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) 	_param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) 	_param_mc_acro_expo,
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) 	_param_mc_acro_expo_y,
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) 	_param_mc_acro_supexpo,
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,
		(ParamFloat<px4::params::MC_RATT_TH>) 		_param_mc_ratt_th,
		(ParamBool<px4::params::MC_BAT_SCALE_EN>) 	_param_mc_bat_scale_en,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) 	_param_mpc_manthr_min,
		(ParamInt<px4::params::MPC_THR_CURVE>) 		_param_mpc_thr_curve,
		(ParamInt<px4::params::MC_AIRMODE>) 		_param_mc_airmode,
		(ParamInt<px4::params::CBRK_RATE_CTRL>) 	_param_cbrk_rate_ctrl
	);

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500_ms;
	/** number of tries before switching to a failsafe flight task */
	static constexpr int NUM_FAILURE_TRIES = 10;
	/** If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200_ms;
	/** During smooth-takeoff, below ALTITUDE_THRESHOLD the yaw-control is turned off ant tilt is limited */
	static constexpr float ALTITUDE_THRESHOLD = 0.3f;
	// ******************************************************************************* //

	// uORB Definitions
	uORB::Publication<landing_gear_s>						_landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<freq_control_s>						_freq_pub{ORB_ID(freq_control)};
	uORB::Publication<vehicle_local_position_setpoint_s>	_local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s>	_traj_sp_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<tracking_errors_s>					_errors_pub{ORB_ID(tracking_errors)};
    uORB::PublicationMulti<rate_ctrl_status_s>				_controller_status_pub{ORB_ID(rate_ctrl_status), ORB_PRIO_DEFAULT};
    uORB::PublicationQueued<vehicle_command_s> 				_pub_vehicle_command{ORB_ID(vehicle_command)};
	uORB::PublicationMulti<actuator_outputs_s> _outputs_pub{ORB_ID(actuator_outputs), ORB_PRIO_DEFAULT};
	
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription _local_pos_sub				{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub			{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub	{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _control_mode_sub			{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _parameter_update_sub		{ORB_ID(parameter_update)};
	uORB::Subscription _att_sub						{ORB_ID(vehicle_attitude)};
	uORB::Subscription _home_pos_sub				{ORB_ID(home_position)};
    uORB::Subscription _battery_status_sub			{ORB_ID(battery_status)};
    uORB::Subscription _motor_limits_sub			{ORB_ID(multirotor_motor_limits)};

	orb_advert_t 	_mavlink_log_pub{nullptr};
	orb_advert_t	_actuators_0_pub{nullptr};

    orb_id_t 	_actuators_id{nullptr};
	// ******************************************************************************* //

	// Functions
	// Update our local parameter cache.
	int parameters_update(bool force);

	// Check for changes in subscribed topics.
	void poll_subscriptions();

	// Check for validity of positon/velocity states.
	void set_vehicle_states(const float &vel_sp_z);

	// Limit altitude based on land-detector.
	void limit_altitude(vehicle_local_position_setpoint_s &setpoint);

	// Prints a warning message at a lowered rate.
	void warn_rate_limited(const char *str);

	// Adjust the setpoint during landing.
	void limit_thrust_during_landing(vehicle_local_position_setpoint_s &setpoint);

	// Start flightasks based on navigation state.
	void start_flight_task();

	/**
	 * Failsafe.
	 * If flighttask fails for whatever reason, then do failsafe. This could
	 * occur if the commander fails to switch to a mode in case of invalid states or
	 * setpoints. The failsafe will occur after LOITER_TIME_BEFORE_DESCEND. If force is set
	 * to true, the failsafe will be initiated immediately.
	 */
	void failsafe(vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states, const bool force, const bool warn);

	// Reset setpoints to NAN
	void reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint);

	// Shim for calling task_main from task_create.
	static int task_main_trampoline(int argc, char *argv[]);

	// Check if task should be switched because of failsafe
	void check_failure(bool task_failure, uint8_t nav_state);

	// Send vehicle command to inform commander about failsafe
	void send_vehicle_cmd_do(uint8_t nav_state);

	// Main sensor collection task.
	void task_main();

	// Polling on Topics
	bool vehicle_attitude_poll();
	void vehicle_motor_limits_poll();
	void vehicle_status_poll();

	// Publish on Topics
    void publish_actuator_controls();
	void publish_rate_controller_status();
	// ******************************************************************************* //

	// Performance Counter
	perf_counter_t _cycle_perf;
};
