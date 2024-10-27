#define SERIAL_SPEED          1000000
#define DEBUG_TIME            false

// Leds
#define LED_NUMBER  5
#define LED1_PIN    4
#define LED2_PIN    5
#define LED3_PIN    6
#define LED4_PIN    7
#define LED5_PIN    11

// Pins des interrupteurs
#define INTERRUPTEUR_START_PIN  2
#define INTERRUPTEUR_MODE_PIN   3

// Pins des moteurs
#define MOTOR_LEFT_PIN_INA1   8
#define MOTOR_LEFT_PIN_INA2   9
#define MOTOR_RIGHT_PIN_INA1  12
#define MOTOR_RIGHT_PIN_INA2  10

#define ENCODER_LEFT_PIN_A    17
#define ENCODER_LEFT_PIN_B    16
#define ENCODER_RIGHT_PIN_A   14
#define ENCODER_RIGHT_PIN_B   15

// Paramètres
#define MOTOR_DEADZONE        0.0

#define DELTA_TIME_S          0.010  // Periode d'échantillonnage [s]
#define DELTA_TIME_MS         DELTA_TIME_S * 1000.0  // Periode d'échantillonnage [ms]

// Moteur : réduction 30:1 vitesse max 530rpm.
// Codeur : 14 tops par tour, donc 420 par tour de roue!
#define N_TOP_PER_WHEEL_TURN  2*420.0   // Nombre de top par tour de roue [-]

#define DIAMETER_WHEEL        0.030 // Diamètre de la roue [m]
#define DIAMETER_WHEEL_LEFT   0.030 // Diamètre de la roue gauche [m]
#define DIAMETER_WHEEL_RIGHT  0.030 // Diamètre de la roue gauche [m]
// Pour un tour de roue, on fait PI*D_WHEEL = 9.43 cm, pour faire un mètre il faut N_TOP_PER_WHEEL_TURN / (PI*D_WHEEL) = 4456.3 tops ou 8912.7 avec le coeff *2
#define DIAMETER_ROBOT        0.10 * 2.3  // Diamètre de rotation du Pami [m]
// Pour un tour de robot, on fait PI* D_ROBOT = 31.4 cm, donc /9.43 = 3.33 tour de roue, c'est à dire 1399 tops ou 2798 tops avec le coeff
#define CORRECTION_DISTANCE   1.0

#define DEG_TO_RAD            PI / 180.0
#define RAD_TO_DEG            180.0 / PI

// TEMPS DU MATCH
#define DUREE_ATTENTE_S       3     // Durée d'attente avant le départ PAMI, ce sera 85 secondes
#define DUREE_ATTENTE_MS      DUREE_ATTENTE_S * 1000

#define DUREE_MATCH_S         100   // Durée d'un match
#define DUREE_MATCH_MS        DUREE_MATCH_S * 1000
