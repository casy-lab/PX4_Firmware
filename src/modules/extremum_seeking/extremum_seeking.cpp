/*
 * @author Lorenzo Gentilini (University of Bologna)
 * May, 2020
 */

#include "extremum_seeking.hpp"

hrt_abstime startTime{};

/** Extremum Seeking Class **/
// Constructor
ExSeeking::ExSeeking(){
	zPlus 	= 0.0;
	z 		= 0.0;
	fPlus 	= 0.0;
	f 		= 0.0;
	yZ 		= 0.0;
	uX		= 0.0;
	uXPlus	= 0.0;
	uY		= 0.0;
	uYPlus	= 0.0;
	time	= 0.0;
}

// Destructor
ExSeeking::~ExSeeking(){;}

// Main Functions
matrix::Matrix<double, 2, 1> ExSeeking::update(double y){
	matrix::Matrix<double, 2, 1> x;

	time = (hrt_absolute_time() - startTime)/1e6;

	// Update Z
	zPlus = A*z + B*y;
    yZ = -C*z + C*y;

	// Update alpha (Reference pre-filtering)
	fPlus = FA*f + FB*ALPHA;
    alpha = FC*f + FD*ALPHA;

	// X - Direction
    uXPlus = uX + D*(yZ*alpha*cos(OMEGA*time));
	x(0,0) = uX + E*(yZ*alpha*cos(OMEGA*time)) + alpha*cos(OMEGA*time);

    // Y - Direction
    uYPlus = uY + D*(yZ*alpha*sin(OMEGA*time));
    x(1,0) = uY + E*(yZ*alpha*sin(OMEGA*time)) + alpha*sin(OMEGA*time);

	// Update Memory Variables
    z = zPlus;
    f = fPlus;	
    uX = uXPlus;
    uY = uYPlus;

	return x;
}

void ExSeeking::reset(){
	zPlus 	= 0.0;
	z 		= 0.0;
	fPlus 	= 0.0;
	f 		= 0.0;
	yZ 		= 0.0;
	uX		= 0.0;
	uXPlus	= 0.0;
	uY		= 0.0;
	uYPlus	= 0.0;
	time	= 0.0;
}

// -------------------------------------------------------------- //

/** Bounded Update Rate ES Class **/
// Constructor
ExSeekingBUR::ExSeekingBUR(){
	fPlus 			= 0.0;
	f 				= 0.0;
	alpha 			= 0.0;
	time			= 0.0;
	previousTime	= 0.0;
	xOld 			= 0.0;
	yOld 			= 0.0;
}

// Destructor
ExSeekingBUR::~ExSeekingBUR(){;}

// Main Functions
matrix::Matrix<double, 3, 1> ExSeekingBUR::update(double y){
	matrix::Matrix<double, 3, 1> x;
	
	fPlus = FA_BUR*f + FB_BUR*ALPHA_BUR;
    alpha = FC_BUR*f + FD_BUR*ALPHA_BUR;

	time = (hrt_absolute_time() - startTime)/1e6;
	double deltaTime = time - previousTime;

	// X - Direction
    x(0,0) = xOld + (sqrt(alpha*OMEGA_BUR)*cos(OMEGA_BUR*time + KAPPA_BUR*y))*deltaTime;

    // Y - Direction
    x(1,0) = yOld + (sqrt(alpha*OMEGA_BUR)*sin(OMEGA_BUR*time + KAPPA_BUR*y))*deltaTime;

	// Z - Direction
	x(2,0) = 0.0;

	// Update Memory Variables
    f = fPlus;
	xOld = x(0,0);
	yOld = x(1,0);
	previousTime = time;

	return x;
}

void ExSeekingBUR::reset(){
	fPlus 			= 0.0;
	f 				= 0.0;
	alpha 			= 0.0;
	time			= 0.0;
	previousTime	= 0.0;
	xOld 			= 0.0;
	yOld 			= 0.0;
}

// -------------------------------------------------------------- //

// ** ESModule Class ** //
// Constructor
ESModule::ESModule() : ModuleParams(nullptr){
	_ref_gen = new ExSeekingBUR;

	iRp(0,0) = 0.5770;
	iRp(0,1) = -0.7071;
	iRp(0,2) = -0.4087;

	iRp(1,0) = 0.5770;
	iRp(1,1) = 0.7071;
	iRp(1,2) = -0.4087;

	iRp(2,0) = 0.5780;
	iRp(2,1) = 0.0;
	iRp(2,2) = 0.8161;

	Op(0,0) = 0.0;
	Op(1,0) = 0.0;
	Op(2,0) = 6.1268;
}

//Destructor
ESModule::~ESModule(){
	delete _ref_gen;
}

// Main Module Operation --- Takeoff and Search --- //
void ESModule::run(){
	PX4_INFO("Extremum Seeking Module Start");

	while(!should_exit()){
		actualTime = hrt_absolute_time();
		_vehicle_control_mode_sub.update(&_mode);

		// Update at Wanted Frequency
		if(actualTime - previousTime >= ((double)1/FREQ)*1e6 && _mode.flag_control_search_enabled && _mode.flag_armed){
			_freq.freq = actualTime - previousTime;
			_freq.timestamp = actualTime;
			_freq_pub.publish(_freq);
			previousTime = actualTime;

			// Update Info from Topics
			_local_pos_sub.update(&_local_pos);

			if(_arva_sub.update(&_sens_arva)){
				if(isFirstTime){
					f = ((1-FD_ARVA)*(double)_sens_arva.y)/FC_ARVA;
					isFirstTime = false;
				}

				// Update ARVA
				fPlus = FA_ARVA*f + FB_ARVA*(double)_sens_arva.y;
    			_arva_filtered = FC_ARVA*f + FD_ARVA*(double)_sens_arva.y;

				f = fPlus;
			}
			// -- //

			// State Machine For TakeOff and ES Search
			switch (_state){
				case TAKEOFF:
				// Takeoff Triplet
				_sp_triplet.x = 0.0;
				_sp_triplet.y = 0.0;
				_sp_triplet.z = -Z_REF;
				_sp_triplet.timestamp = hrt_absolute_time();

				if(abs((double)_local_pos.z + Z_REF) < EPSILON){
					count++;

					if(count == (int)200){
						_state = SEARCH;
						startTime = hrt_absolute_time();
					}

				}else {
					count = 0;
				}

				break;

				case SEARCH:
					// Search Triplet
					matrix::Matrix<double, 3, 1> sp_p = _ref_gen->update(_arva_filtered);

					// Search On Plane
					matrix::Matrix<double, 3, 1> sp_i = Op + iRp*sp_p;

					_sp_triplet.x 	= sp_i(0,0);
					_sp_triplet.y 	= sp_i(1,0);
					_sp_triplet.z 	= -sp_i(2,0);
					_sp_triplet.timestamp = hrt_absolute_time();
				break;
			}

			// Publish
			_sp_triplet_pub.publish(_sp_triplet);
		}
	}
}

// Module Functions --------------------------------------------------------- //
int ESModule::print_status(){
	PX4_INFO("Running");

	return 0;
}

int ESModule::custom_command(int argc, char *argv[])
{
	if (!is_running()){
		print_usage("Module Not Running");
		return 1;
	}

	return print_usage("Unknown Command");
}

// To Spawn The Application
int ESModule::task_spawn(int argc, char *argv[]){
	_task_id = px4_task_spawn_cmd("es_module", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT-10,
				      1800, (px4_main_t)&run_trampoline, (char *const *)argv);

	if (_task_id < 0){
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ESModule *ESModule::instantiate(int argc, char *argv[]){
	ESModule *instance = new ESModule();

	if (instance == nullptr) {
		PX4_ERR("Alloc Failed");
	}

	return instance;
}

int ESModule::print_usage(const char *reason){
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
		### Description
		Extremum Seeking Module.

		This module work with 'start/stop/status' functionality.
		Type 'start' to begin the searching.

		)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("extremum_seeking", "");
	PRINT_MODULE_USAGE_COMMAND("'start'");

	return 0;
}

int extremum_seeking_main(int argc, char *argv[]){
	return ESModule::main(argc, argv);
}
