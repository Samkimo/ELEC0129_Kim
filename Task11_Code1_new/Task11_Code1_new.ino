#include <Servo.h>

/* ============================================================================
 * Task 11 - Code 1: Manual Coordinate Input
 * ============================================================================ */

// Robot Link Dimensions (Unit: cm, matching the hand-drawn diagram exactly)
const float L1 = 0.0;  // Base to Shoulder Pitch (Joint 1 to Joint 2) offset
const float L2 = 9.6;  // Shoulder to Elbow (Joint 2 to Joint 3)
const float Lg = 14.9; // Elbow to Gripper (Joint 3 to Gripper end)

const float offset_theta1= 7;
const float offset_theta2 = 2;
const float offset_theta3 =-5;
// Servo Objects
Servo servo1; // Base Yaw
Servo servo2; // Shoulder Pitch
Servo servo3; // Elbow Pitch

// Array storing 5 predefined target coordinates {X, Y, Z} in cm
// TODO: Replace with your actual tested and safe coordinates
// const float targets[5][3] = {
//   {14.0, 0.0, 9.5},    // Target 1
//   {7.0, -14.0, 18.0},   // Target 2
//   {3.0, -7.0, 21.0},  // Target 3
//   {5.0, 3.0, 22.5},   // Target 4
//   {8, 17.0, 14.0}     // Target 5
// };
 const float targets[5][3] = {
   {14.9, 0.0, 9.6},        // Target 1
   {7.4, -13.6, 18.20},   // Target 2
   {3.6, -6.6, 22.2},    // Target 3
   {4.8, 4.3, 23.6},      // Target 4
   {9.6, 16.2, 13.0}     // Target 5
 };


int currentTarget = 0; // Index to keep track of the current target

/*
 * Inverse Kinematics Function
 */
void moveToPosition(float x, float y, float z) {
  // 1. Base Angle
  float theta1_rad = atan2(y, x)+PI/2; //offset 
  
  // 2. Reduce to 2D planar problem
  float R = sqrt(pow(x, 2) + pow(y, 2)); 
  float z_prime = z - L1; 
  
  // 3. Elbow Angle (Joint 3) using Law of Cosines
  // Replaced old L1/L2 with new L2/Lg to match the diagram
  float c3 = constrain((pow(R, 2) + pow(z_prime, 2) - pow(L2, 2) - pow(Lg, 2)) / (2.0 * L2 * Lg), -1.0, 1.0);
  
  // Keeping only the "Elbow-up" solution
  float s3 = -sqrt(1.0 - pow(c3, 2)); 
  float theta3_rad = -atan2(s3, c3); 
  
  // 4. Shoulder Angle (Joint 2)
  float K1 = L2 + Lg * c3;
  float K2 = Lg * s3;
  float beta = atan2(K2,K1);
  float theta2_rad = atan2(z_prime,R)-beta;
  //float c2 = (K1 * R - K2 * z_prime) / (pow(K1, 2) + pow(K2, 2));
  //float s2 = (-K2 * R - K1 * z_prime) / (pow(K1, 2) + pow(K2, 2));
  //float theta2_rad = atan2(s2, c2); 
  
  // 5. Convert radians to degrees
  int angle1 = round(theta1_rad * 180.0 / PI)+offset_theta1;
  int angle2 = round(theta2_rad * 180.0 / PI)+offset_theta2;
  int angle3 = round(theta3_rad * 180.0 / PI)+offset_theta3;

  int angle1_clear = round(theta1_rad * 180.0 / PI);
  int angle2_clear = round(theta2_rad * 180.0 / PI);
  int angle3_clear = round(theta3_rad * 180.0 / PI);

  // 6. Actuate servos
  servo1.write(constrain(angle1, 0, 180));
  servo2.write(constrain(angle2, 0, 180));
  servo3.write(constrain(angle3, 0, 180));
  
  // Print results
  Serial.print("Target XYZ (cm): ("); Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", "); Serial.print(z); Serial.println(")");
  Serial.print("Angles: T1="); Serial.print(angle1_clear);
  Serial.print(" T2="); Serial.print(angle2_clear);
  Serial.print(" T3="); Serial.println(angle3_clear);
  Serial.println("-------------------------");
}

void setup() {
  Serial.begin(115200);
  
  // Attach servos
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  
  delay(2000);
}

void loop() {
  float targetX = targets[currentTarget][0];
  float targetY = targets[currentTarget][1];
  float targetZ = targets[currentTarget][2];
  
  moveToPosition(targetX, targetY, targetZ);
  
  delay(2000); // 3-second delay between points
  
  currentTarget++;
  if (currentTarget >= 5) {
    currentTarget = 0; 
  }
} 