#ifndef config_meca_h_
#define config_meca_h_

/******************************************************************************
   This is the mechanical configuration of the ROBOT_CORE
 ******************************************************************************/
#ifdef ROBOT_CORE

#define MOTOR_DEADZONE        4.0

/*
 * The ROBOT_CORE motor has a reduction ratio of 30:1 and a maximum speed of 220 rpm.
 * The encoder has 7 tops per revolution. Read in quadrature, this makes 7x4=28 
 * tops per motor revolution. With the reduction ratio this makes 28x30 = 840
 * tops per wheel rotation.
 */
#define N_TOP_PER_WHEEL_TURN  2*500.0    /* Encoder top number per wheel turn [-] */
#define FACTOR_DISTANCE       0.9308820107051 /* Factor to adjust the top count */
#define FACTOR_WHEEL_LEFT     1.0
#define FACTOR_WHEEL_RIGHT    1.0
/*
 * The Pami wheels are 4cm or 0.04m in diameter. The perimeter is PI*0.04=0.126m,
 * which is 1/(PI*0.04)= 8 wheel rotations to travel 1m. To convert in top number 
 * it requires 4200.0 / (PI*0.04) tops for 1m.
 */
#define WHEEL_DIAMETER_M      0.040     /* Wheel diameter [m] */
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN * FACTOR_DISTANCE / (WHEEL_DIAMETER_M * PI);
#define METER_TO_TOP          (7957.747154 * FACTOR_DISTANCE)

/*
 * To determine the distance needed to rotate of a certain angle, one has to 
 * consider the wheelbase, or distance between the wheels. The robot rotates
 * along a circle of this diameter. One full rotation is PI*wheelbase and the
 * angle must be in radians, which is 2*PI for a full rotation. So the distance
 * needed for an angle A is A/2*wheelbase and to convert it in top, use formula
 * above to get A/2*Ntop*Wheelbase/Wheeldiameter/PI. This distance is needed on both
 * wheel, so *2.
 */
#define WHEELBASE_M           0.250 * 0.99802005834  /* Wheelbase of the robot [m] */
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (WHEEL_DIAMETER_M / WHEELBASE_M ) / PI
#define RAD_TO_TOP            (1989.4367886486 * FACTOR_DISTANCE)

#define BACK_LENGTH           0.0 /* Length between wheel center and the back of the ROBOT_CORE */

#endif 

#endif
