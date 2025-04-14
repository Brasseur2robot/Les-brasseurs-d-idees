#ifndef config_meca_h_
#define config_meca_h_

/******************************************************************************
   This is the mechanical configuration of Zophon's Sumo robot
 ******************************************************************************/
#ifdef SUMO_ZOPHON

#define MOTOR_DEADZONE        0.0

/*
 * The Sumo motor has a reduction ratio of 30:1 and maximum speed of 530 rpm.
 * The encoder has 7 tops per revolution. Read in quadrature, this makes 7x4=28 
 * tops per motor revolution. With the reduction ratio this makes 28x30 = 840
 * tops per wheel turn.
 */
#define N_TOP_PER_WHEEL_TURN  840.0         /* Encoder top number per wheel turn [-] */

/*
 * The Sumo wheels are 3cm or 0.03m in diameter. The perimeter is PI*0.03=0.094m,
 * which is 1/(PI*0.03)= 10.6 wheel rotations to travel 1m. To convert in top number 
 * it requires 840.0 / (PI*0.03) tops for 1m.
 */
#define DIAMETER_WHEEL        0.030         /* Wheel diameter [m] */
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL * PI);
#define METER_TO_TOP          8912.6768131461

/*
 * To determine the distance needed to rotate of a certain angle, one has to 
 * consider the wheelbase, or distance between the wheels. The robot rotates
 * along a circle of this diameter. One full rotation is PI*wheelbase and the
 * angle must be in radians, which is 2*PI for a full rotation. So the distance
 * needed for an angle A is A/2*wheelbase and to convert it in top, use formula
 * above to get A/2*Ntop*Wheelbase/Wheeldiameter/PI. This distance is needed on both
 * wheel, so *2.
 */
#define DIAMETER_ROBOT        0.10          /* Wheelbase of the pami [m] */
// Pour un tour de robot, on fait PI* D_ROBOT = 31.4 cm, donc /9.43 = 3.33 tour de roue, c'est à dire 1399 tops ou 2798 tops avec le coeff
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL / DIAMETER_ROBOT ) / PI
#define RAD_TO_TOP            (891.2676813146 * 1.23)

#endif

/******************************************************************************
   This is the mechanical configuration of the PAMI 1
 ******************************************************************************/
#ifdef PAMI_1

#define MOTOR_DEADZONE        24.0

/*
 * The Pami motor has a reduction ratio of 30:1 and a maximum speed of 220 rpm.
 * The encoder has 7 tops per revolution. Read in quadrature, this makes 7x4=28 
 * tops per motor revolution. With the reduction ratio this makes 28x30 = 840
 * tops per wheel rotation.
 */
#define N_TOP_PER_WHEEL_TURN  840.0    /* Encoder top number per wheel turn [-] */
#define FACTOR_DISTANCE       1.0 //0.9866798224 /* Factor to adjust the top count */
#define FACTOR_WHEEL_LEFT     1.00061525053506 // 1.00061525
#define FACTOR_WHEEL_RIGHT    0.999384749464942 // 0.99938475
/*
 * The Pami wheels are 4cm or 0.04m in diameter. The perimeter is PI*0.04=0.126m,
 * which is 1/(PI*0.04)= 8 wheel rotations to travel 1m. To convert in top number 
 * it requires 4200.0 / (PI*0.04) tops for 1m.
 */
#define WHEEL_DIAMETER_M      0.038     /* Wheel diameter [m] */
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN * FACTOR_DISTANCE / (WHEEL_DIAMETER_M * PI);
#define METER_TO_TOP          (6868.0527 * FACTOR_DISTANCE)

/*
 * To determine the distance needed to rotate of a certain angle, one has to 
 * consider the wheelbase, or distance between the wheels. The robot rotates
 * along a circle of this diameter. One full rotation is PI*wheelbase and the
 * angle must be in radians, which is 2*PI for a full rotation. So the distance
 * needed for an angle A is A/2*wheelbase and to convert it in top, use formula
 * above to get A/2*Ntop*Wheelbase/Wheeldiameter/PI. This distance is needed on both
 * wheel, so *2.
 */
#define WHEELBASE_M           0.079 // 0.08589065014   /* Wheelbase of the pami [m] */
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (WHEEL_DIAMETER_M / WHEELBASE_M ) / PI
#define RAD_TO_TOP            540.527608239969 //(548.4652985688 * 0.99391394) // 545.12730573

#define BACK_LENGTH           0.057 /* Length between wheel center and the back of the PAMI */

#endif

/******************************************************************************
   This is the mechanical configuration of the PAMI 2
 ******************************************************************************/
#ifdef PAMI_2

#define MOTOR_DEADZONE        20.0

/*
 * The Pami motor has a reduction ratio of 30:1 and a maximum speed of 220 rpm.
 * The encoder has 7 tops per revolution. Read in quadrature, this makes 7x4=28 
 * tops per motor revolution. With the reduction ratio this makes 28x30 = 840
 * tops per wheel rotation.
 */
#define N_TOP_PER_WHEEL_TURN  840.0    /* Encoder top number per wheel turn [-] */
#define FACTOR_DISTANCE       1.0 /* Factor to adjust the top count */
#define FACTOR_WHEEL_LEFT     1.0 //0.99906059 * 1.001760657
#define FACTOR_WHEEL_RIGHT    1.0 //1.00093941 * 0.998239343
/*
 * The Pami wheels are 4cm or 0.04m in diameter. The perimeter is PI*0.04=0.126m,
 * which is 1/(PI*0.04)= 8 wheel rotations to travel 1m. To convert in top number 
 * it requires 4200.0 / (PI*0.04) tops for 1m.
 */
#define WHEEL_DIAMETER_M      0.038     /* Wheel diameter [m] */
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN * FACTOR_DISTANCE / (WHEEL_DIAMETER_M * PI);
#define METER_TO_TOP          6868.0527

/*
 * To determine the distance needed to rotate of a certain angle, one has to 
 * consider the wheelbase, or distance between the wheels. The robot rotates
 * along a circle of this diameter. One full rotation is PI*wheelbase and the
 * angle must be in radians, which is 2*PI for a full rotation. So the distance
 * needed for an angle A is A/2*wheelbase and to convert it in top, use formula
 * above to get A/2*Ntop*Wheelbase/Wheeldiameter/PI. This distance is needed on both
 * wheel, so *2.
 */
#define WHEELBASE_M           0.079   /* Wheelbase of the pami [m] */
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (WHEEL_DIAMETER_M / WHEELBASE_M ) / PI
#define RAD_TO_TOP            (577.3776988 * 0.9380487687499 * FACTOR_DISTANCE)

#define BACK_LENGTH           0.057 /* Length between wheel center and the back of the PAMI */

#endif

/******************************************************************************
   This is the mechanical configuration of the PAMI 3
 ******************************************************************************/
 #ifdef PAMI_3

 #define MOTOR_DEADZONE        20.0
 
 /*
  * The Pami motor has a reduction ratio of 30:1 and a maximum speed of 220 rpm.
  * The encoder has 7 tops per revolution. Read in quadrature, this makes 7x4=28 
  * tops per motor revolution. With the reduction ratio this makes 28x30 = 840
  * tops per wheel rotation.
  */
 #define N_TOP_PER_WHEEL_TURN  840.0    /* Encoder top number per wheel turn [-] */
 #define FACTOR_DISTANCE       1.0 /* Factor to adjust the top count */
 #define FACTOR_WHEEL_LEFT     0.99906059 * 1.001760657
 #define FACTOR_WHEEL_RIGHT    1.00093941 * 0.998239343
 /*
  * The Pami wheels are 4cm or 0.04m in diameter. The perimeter is PI*0.04=0.126m,
  * which is 1/(PI*0.04)= 8 wheel rotations to travel 1m. To convert in top number 
  * it requires 4200.0 / (PI*0.04) tops for 1m.
  */
 #define WHEEL_DIAMETER_M      0.038     /* Wheel diameter [m] */
 //#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN * FACTOR_DISTANCE / (WHEEL_DIAMETER_M * PI);
 #define METER_TO_TOP          (6868.0527 * FACTOR_DISTANCE)
 
 /*
  * To determine the distance needed to rotate of a certain angle, one has to 
  * consider the wheelbase, or distance between the wheels. The robot rotates
  * along a circle of this diameter. One full rotation is PI*wheelbase and the
  * angle must be in radians, which is 2*PI for a full rotation. So the distance
  * needed for an angle A is A/2*wheelbase and to convert it in top, use formula
  * above to get A/2*Ntop*Wheelbase/Wheeldiameter/PI. This distance is needed on both
  * wheel, so *2.
  */
 #define WHEELBASE_M           0.08589065014   /* Wheelbase of the pami [m] */
 //#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (WHEEL_DIAMETER_M / WHEELBASE_M ) / PI
 #define RAD_TO_TOP            (577.3776988 * 1.050614621 * FACTOR_DISTANCE)
 
 #define BACK_LENGTH           0.057 /* Length between wheel center and the back of the PAMI */
 
 #endif


/* #ifdef PAMI_3

#define MOTOR_DEADZONE        20.0


 * The Pami motor has a reduction ratio of 50:1 and a maximum speed of X rpm.
 * The encoder has 7 tops per revolution. Read in quadrature, this makes 7x4=28 
 * tops per motor revolution. With the reduction ratio this makes 28x50 = 840
 * tops per wheel rotation.
 
#define N_TOP_PER_WHEEL_TURN  1400.0    Encoder top number per wheel turn [-]
#define FACTOR_DISTANCE       1.0       Factor to adjust the top count
#define FACTOR_WHEEL_LEFT     1.0
#define FACTOR_WHEEL_RIGHT    1.0

 * The Pami wheels are 4cm or 0.04m in diameter. The perimeter is PI*0.04=0.126m,
 * which is 1/(PI*0.04)= 8 wheel rotations to travel 1m. To convert in top number 
 * it requires 4200.0 / (PI*0.04) tops for 1m.

#define WHEEL_DIAMETER_M      0.038     Wheel diameter [m]
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN * FACTOR_DISTANCE / (WHEEL_DIAMETER_M * PI);
#define METER_TO_TOP          (11727.206331 * FACTOR_DISTANCE)


 * To determine the distance needed to rotate of a certain angle, one has to 
 * consider the wheelbase, or distance between the wheels. The robot rotates
 * along a circle of this diameter. One full rotation is PI*wheelbase and the
 * angle must be in radians, which is 2*PI for a full rotation. So the distance
 * needed for an angle A is A/2*wheelbase and to convert it in top, use formula
 * above to get A/2*Ntop*Wheelbase/Wheeldiameter/PI. This distance is needed on both
 * wheel, so *2.
 
#define WHEELBASE_M           0.086    Wheelbase of the pami [m] 
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (WHEEL_DIAMETER_M / WHEELBASE_M ) / PI
#define RAD_TO_TOP            (1008.539745 * FACTOR_DISTANCE)

#define BACK_LENGTH           0.057  Length between wheel center and the back of the PAMI 

#endif
*/

/******************************************************************************
   This is the mechanical configuration of the PAMI 4
 ******************************************************************************/
#ifdef PAMI_4

#define MOTOR_DEADZONE        20.0

/*
 * The Pami motor has a reduction ratio of 30:1 and a maximum speed of 220 rpm.
 * The encoder has 7 tops per revolution. Read in quadrature, this makes 7x4=28 
 * tops per motor revolution. With the reduction ratio this makes 28x30 = 840
 * tops per wheel rotation.
 */
#define N_TOP_PER_WHEEL_TURN  840.0    /* Encoder top number per wheel turn [-] */
#define FACTOR_DISTANCE       1.0 /* Factor to adjust the top count */
#define FACTOR_WHEEL_LEFT     1.0 //0.99906059 * 1.001760657
#define FACTOR_WHEEL_RIGHT    1.0 //1.00093941 * 0.998239343
/*
 * The Pami wheels are 4cm or 0.04m in diameter. The perimeter is PI*0.04=0.126m,
 * which is 1/(PI*0.04)= 8 wheel rotations to travel 1m. To convert in top number 
 * it requires 4200.0 / (PI*0.04) tops for 1m.
 */
#define WHEEL_DIAMETER_M      0.038     /* Wheel diameter [m] */
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN * FACTOR_DISTANCE / (WHEEL_DIAMETER_M * PI);
#define METER_TO_TOP          (6868.0527 * FACTOR_DISTANCE)

/*
 * To determine the distance needed to rotate of a certain angle, one has to 
 * consider the wheelbase, or distance between the wheels. The robot rotates
 * along a circle of this diameter. One full rotation is PI*wheelbase and the
 * angle must be in radians, which is 2*PI for a full rotation. So the distance
 * needed for an angle A is A/2*wheelbase and to convert it in top, use formula
 * above to get A/2*Ntop*Wheelbase/Wheeldiameter/PI. This distance is needed on both
 * wheel, so *2.
 */
#define WHEELBASE_M           0.079   /* Wheelbase of the pami [m] */
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (WHEEL_DIAMETER_M / WHEELBASE_M ) / PI
#define RAD_TO_TOP            (577.3776988 * 1.050614621 * FACTOR_DISTANCE)

#define BACK_LENGTH           0.057 /* Length between wheel center and the back of the PAMI */

#endif 

#endif