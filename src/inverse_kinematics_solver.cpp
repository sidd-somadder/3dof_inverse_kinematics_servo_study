#include <Servo.h>
#include <Arduino.h>

// Initialize the three servo objects
Servo SERVO_B;
Servo SERVO_S;
Servo SERVO_E;

// Save fixed arm link lengths as global constants
const double link_A = 0.2;
const double link_B = 0.15;

// For testing, use fixed target coordinates
const double x_p = 0.1;
const double y_p = 0.15;
const double z_p = 0.1;
// Calculate radius of target point from origin (the shoulder in this case)
double r = sqrt(sq(x_p) + sq(y_p) + sq(z_p));

void setup() {
    // Attach base, shoulder, and elbow servo to respective PWM pins on Arduino/ESP32
    SERVO_B.attach(9);
    SERVO_S.attach(10);
    SERVO_E.attach(11);

    // Set baud rate
    Serial.begin(9600);
}

void loop() {
    // Calculate SERVO_B (base) angle using target coordinates
    double theta_B = rad2deg(atan2(y_p, x_p));

    // Calculate SERVO_S (shoulder) angle using law of cosines
    double theta_S = rad2deg(get_shoulder_angle());
    
    //Calculate SERVO_E (elbow) angle using law of cosines
    double theta_E = rad2deg(get_elbow_angle());

    Serial.print("Base Angle: ");
    Serial.println(theta_B);
    Serial.print("Shoulder Angle: ");
    Serial.println(theta_S);
    Serial.print("Elbow Angle: ");
    Serial.println(theta_E);
    
    delay(4000);
}

double rad2deg(double radians){
    return radians * (180/(PI));
}

double get_elbow_angle(){
    // D,E = cos(alpha),sin(alpha) where alpha is the interior angle between the two arm links
    double D = (sq(link_A) + sq(link_B) - sq(r)) / (2 * link_A * link_B);
    double E = sqrt(1 - sq(D));
    
    // divide E/D for tan(alpha) and get alpha in radians with arctangent.
    double alpha = atan2(E,D);

    // theta_E is the difference between pi & interior angle alpha.
    return PI - alpha;
}

double get_shoulder_angle(){
    // Calculate target vector length when projected onto xy plane
    double ground_dist = sqrt(sq(x_p) + sq(y_p));
    
    // Calculate between target vector and xy plane
    double beta = atan2(z_p, ground_dist);

    // K,L = cos(gamma),sin(gamma) where gamma is the interior angle between the target vector and the first arm link.
    double K = (sq(link_A) + sq(r) - sq(link_B)) / (2 * link_A * r);
    double L = sqrt(1 - sq(K));

    // divide K/L for tan(alpha) and get alpha in radians with arctangent.
    double gamma = atan2(K,L);

    // theta_S is the difference between the angles between target vector and the ground and between arm link 1 and the target vector 
    return beta - gamma;
}