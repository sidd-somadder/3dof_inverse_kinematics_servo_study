#include <Servo.h>
#include <Arduino.h>

// Initialize the three servo objects
Servo SERVO_B;
Servo SERVO_S;
Servo SERVO_E;

// Save fixed arm link lengths as global constants (cm)
const double link_A = 20;
const double link_B = 15;

// Save starting angle which servos move to on startup; for now 90 deg.
const double START_ANG = 90;

// For testing, use fixed target coordinates (cm)
const double x_p = 0;
const double y_p = 0;
const double z_p = 35;

void setup() {
    // Attach base, shoulder, and elbow servo to respective PWM pins on Arduino/ESP32
    SERVO_B.attach(9);
    SERVO_S.attach(10);
    SERVO_E.attach(11);

    // On start up, all three servos start at 90 degrees, 
    SERVO_B.write(START_ANG);
    SERVO_S.write(START_ANG);
    SERVO_E.write(START_ANG);

    // Set baud rate
    Serial.begin(9600);

    delay(500);
}

void loop() {
    if (!isReachable(x_p, y_p, z_p)) {
        Serial.println("Target unreachable. Holding previous position.");
        Serial.print("---");
        delay(4000);
        return;
    }

    // Calculate radius of target point from origin (the shoulder in this case)
    double r = sqrt(sq(x_p) + sq(y_p) + sq(z_p));

    // Calculate SERVO_B (base) angle using target coordinates
    double theta_B = rad2deg(atan2(y_p, x_p));

    // Calculate SERVO_S (shoulder) angle using law of cosines
    double theta_S = rad2deg(getShoulderAngle(r, x_p, y_p));
    
    //Calculate SERVO_E (elbow) angle using law of cosines
    double theta_E = rad2deg(getElbowAngle(r));

    Serial.print("Base Angle: ");
    Serial.println(theta_B);
    Serial.print("Shoulder Angle: ");
    Serial.println(theta_S);
    Serial.print("Elbow Angle: ");
    Serial.println(theta_E);
    Serial.print("---");
    delay(4000);

    // Map & constrain angles for each servo's output, rounding since 
    // Servo.write() uses truncation on the raw input instead, introducing error
    int servoB_cmd = constrain((int)round(theta_B + 90), 0, 180);
    int servoS_cmd = constrain((int)round(theta_S + 90), 0, 180);

    // Accounting for 2:1 gear setup for the base servo, the desired angle is divided by half to constrain 
    // servo output to within (-90 deg, 90 deg)
    int servoE_cmd = 0.5 * constrain((int)round(theta_E), 0, 360);
}

double rad2deg(double radians){
    return radians * (180/(PI));
}

double getElbowAngle(double r){
    // D,E = cos(alpha),sin(alpha) where alpha is the interior angle between the two arm links
    double D = (sq(link_A) + sq(link_B) - sq(r)) / (2 * link_A * link_B);
    double E = sqrt(1 - sq(D));
    
    // divide E/D for tan(alpha) and get alpha in radians with arctangent.
    double alpha = atan2(E,D);

    // theta_E is the difference between pi & interior angle alpha.
    return PI - alpha;
}

// Note that x_p and y_p are currently global doubles, but in the future this wouldn't be the case
double getShoulderAngle(double r, double x_p, double y_p){
    // Calculate target vector length when projected onto xy plane
    double ground_dist = sqrt(sq(x_p) + sq(y_p));
    
    // Calculate between target vector and xy plane
    double beta = atan2(z_p, ground_dist);

    // K,L = cos(gamma),sin(gamma) where gamma is the interior angle between the target vector and the first arm link.
    double K = (sq(link_A) + sq(r) - sq(link_B)) / (2 * link_A * r);
    double L = sqrt(1 - sq(K));

    // divide L/K for tan(alpha) and get alpha in radians with arctangent.
    double gamma = atan2(L,K);

    // theta_S is the difference between the angles between target vector and the ground and between arm link 1 and the target vector 
    return beta - gamma;
}

// check if an target coordinate is even within theoretical reach of the arm; set the origin at the shoulder 
bool isReachable(double x, double y, double z){
    double r = sqrt(sq(x) + sq(y) + sq(z));
    double maxReach = link_A + link_B;
    double minReach = fabs(link_A - link_B);
    return (r <= maxReach && r >= minReach);
}

void moveServosSmooth(int targetB, int targetS, int targetE){
    // Originally, the arm would start with the set start angle
    static int curB = START_ANG, curS = START_ANG, curE = START_ANG;

    // Idea here is that we move at pace with the largest angular difference between desired and saved current angles
    int steps = max(abs(targetB-curB), max(abs(targetS-curS), abs(targetE-curE))); 

    for(int i = 1; i <= steps; i++){
        // Increment towards the desired angle in small steps to avoid sudden sharp loads on the motors
        SERVO_B.write(curB + (long(targetB-curB) * i / steps));
        SERVO_S.write(curS + (long(targetS-curS) * i / steps));
        SERVO_B.write(curE + (long(targetE-curE) * i / steps));
        delayMicroseconds(10);
    }

    curB = targetB;
    curS = targetS;
    curE = targetE;
}