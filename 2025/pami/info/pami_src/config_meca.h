/* Mechanical parameters of the robot */


/******************************************************************************
   This is the mechanical configuration of Zophon's Sumo robot
 ******************************************************************************/
#ifdef SUMO_ZOPHON

#define MOTOR_DEADZONE        0.0

// Moteur : réduction 30:1 vitesse max 530rpm.
// Codeur : 14 tops par tour, donc 420 par tour de roue!
#define N_TOP_PER_WHEEL_TURN  2*420.0   // Nombre de top par tour de roue [-]
#define DIAMETER_WHEEL        0.030 // Diamètre de la roue [m]

// Pour un tour de roue, on fait PI*D_WHEEL = 9.43 cm, pour faire un mètre il faut N_TOP_PER_WHEEL_TURN / (PI*D_WHEEL) = 4456.3 tops ou 8912.7 avec le coeff *2
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL * PI);
#define METER_TO_TOP          8912.676813

#define DIAMETER_ROBOT        0.10 * 2.23  // Diamètre de rotation du Pami [m]
// Pour un tour de robot, on fait PI* D_ROBOT = 31.4 cm, donc /9.43 = 3.33 tour de roue, c'est à dire 1399 tops ou 2798 tops avec le coeff
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL / DIAMETER_ROBOT ) / 2.0 / PI
#define RAD_TO_TOP            (445.633840657 * 2.23)

#endif

/******************************************************************************
   This is the mechanical configuration of a typical PAMI robot
 ******************************************************************************/
#ifdef PAMI

#define MOTOR_DEADZONE        0.0

// Moteur : réduction 150:1 vitesse max 100rpm?
// Codeur : 7*4=28 tops par tour, donc 28*150=4200 par tour de roue!
#define N_TOP_PER_WHEEL_TURN  4200.0    // Nombre de top par tour de roue [-]
#define DIAMETER_WHEEL        0.040     // Diamètre de la roue [m]

// Pour un tour de roue, on fait PI*D_WHEEL = 9.43 cm, pour faire un mètre il faut N_TOP_PER_WHEEL_TURN / (PI*D_WHEEL) = 4456.3 tops ou 8912.7 avec le coeff *2
//#define METER_TO_TOP          N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL * PI);
#define METER_TO_TOP          33422.5380492980

#define DIAMETER_ROBOT        0.083     // Diamètre de rotation du Pami [m]
// Pour un tour de robot, on fait PI* D_ROBOT = cm, donc x tour de roue, c'est à dire  tops ou tops avec le coeff
//#define RAD_TO_TOP            N_TOP_PER_WHEEL_TURN / (DIAMETER_WHEEL / DIAMETER_ROBOT ) / 2.0 / PI
#define RAD_TO_TOP            1387.0353290459

#endif
