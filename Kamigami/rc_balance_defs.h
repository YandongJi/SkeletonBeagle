/**
 * rc_balance_defs.h
 *
 * Contains the settings for configuration of rc_balance.c
 */

#ifndef RC_BALANCE_CONFIG
#define RC_BALANCE_CONFIG

#define SAMPLE_RATE_HZ		100	// main filter and control loop speed
#define DT			0.01			// 1/sample_rate

// Structural properties of eduMiP
#define BOARD_MOUNT_ANGLE	0.49 // increase if mip tends to roll forward
#define GEARBOX			35.577
#define ENCODER_RES		60
#define WHEEL_RADIUS_M		0.034
#define TRACK_WIDTH_M		0.035
#define V_NOMINAL		7.4

// inner loop controller 100hz
#define D1_GAIN			1.05
#define D1_ORDER		2
#define D1_NUM			{-4.945, 8.862, -3.967}
#define D1_DEN			{ 1.000, -1.481, 0.4812}
#define D1_NUM_LEN		3
#define D1_DEN_LEN		3
#define D1_SATURATION_TIMEOUT	0.4


// outer loop controller 100hz
#define D2_GAIN			0.9
#define	D2_ORDER		2
#define D2_NUM			{0.18856,  -0.37209,  0.18354}
#define D2_DEN			{1.00000,  -1.86046,   0.86046}
#define D2_NUM_LEN		3
#define D2_DEN_LEN		3
#define THETA_REF_MAX		0.33

// steering controller
#define D3_KP			1.0
#define D3_KI			0.3
#define D3_KD			0.05
#define STEERING_INPUT_MAX	0.5

// electrical hookups
#define MOTOR_CHANNEL_L		3
#define MOTOR_CHANNEL_R		2
#define MOTOR_POLARITY_L	1
#define MOTOR_POLARITY_R	-1
#define ENCODER_CHANNEL_L	3
#define ENCODER_CHANNEL_R	2
#define ENCODER_POLARITY_L	1
#define ENCODER_POLARITY_R	-1

//	drive speeds when using remote control (dsm2)
#define DRIVE_RATE_NOVICE	16
#define TURN_RATE_NOVICE	6
#define DRIVE_RATE_ADVANCED	26
#define TURN_RATE_ADVANCED	10

// DSM channel config
#define DSM_DRIVE_POL		1
#define DSM_TURN_POL		1
#define DSM_DRIVE_CH		3
#define DSM_TURN_CH		2
#define DSM_DEAD_ZONE		0.04

// Thread Loop Rates
#define BATTERY_CHECK_HZ	5
#define SETPOINT_MANAGER_HZ	100
#define PRINTF_HZ		50

// other
#define TIP_ANGLE		0.85
#define START_ANGLE		0.3
#define START_DELAY		0.4
#define PICKUP_DETECTION_TIME	0.6
#define ENABLE_POSITION_HOLD	1
#define SOFT_START_SEC		0.7


/**
 * NOVICE: Drive rate and turn rate are limited to make driving easier.
 * ADVANCED: Faster drive and turn rate for more fun.
 */
typedef enum drive_mode_t{
	NOVICE,
	ADVANCED
}drive_mode_t;

/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;





/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t{
	double wheelAngleR;	///< wheel rotation relative to body
	double wheelAngleL;
	double theta;		///< body angle radians
	double phi;		///< average wheel angle in global frame
	double gamma;		///< body turn (yaw) angle radians
	double vBatt;		///< battery voltage
	double d1_u;		///< output of balance controller D1 to motors
	double d2_u;		///< output of position controller D2 (theta_ref)
	double d3_u;		///< output of steering controller D3 to motors
	double dutyR, dutyL; // duty cycle for left an dright leg motors
	double mot_drive;	///< u compensated for battery voltage
} core_state_t;


// possible modes, user selected with command line arguments
typedef enum m_input_mode_t{
	NONE,
	DSM,
	STDIN
} m_input_mode_t;

/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.
 */
typedef struct setpoint_t{
	arm_state_t arm_state;	///< see arm_state_t declaration
	drive_mode_t drive_mode;///< NOVICE or ADVANCED
	double theta;		///< body lean angle (rad)
	double phi;		///< wheel position (rad)
	double phi_dot;		///< rate at which phi reference updates (rad/s)
	double gamma;		///< body turn angle (rad)
	double gamma_dot;	///< rate at which gamma setpoint updates (rad/s)
}setpoint_t;



#endif	// endif RC_BALANCE_CONFIG
