#include <math.h>
#include <Servo.h>
/* The space for the target values*/
const float target_values[3][3] = {
  {x0, y0, z0}, // Initial point
  {x1, y1, z1}, // Point 1
  {x2, y2, z2}, // Point 2
}

/* Definition to the link length & basic parameters */
const float L1 = 0.0;  // Base to Shoulder Pitch (Joint 1 to Joint 2) offset
const float L2 = 9.6;  // Shoulder to Elbow (Joint 2 to Joint 3)
const float Lg = 14.9; // Elbow to Gripper (Joint 3 to Gripper end)

/* Define the servo motors */
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo gripperServo;

/* Define the starting angles */
starting_angle1 = 90;
starting_angle2 = 90;
starting_angle3 = 90;
starting_angle_gripper = 0;

/* Inverse kinematics function */
void moveToPosition(float x, float y, float z){
  bool OK = true;
  float theta1_rad = atan2(y,x);
  
  float R = sqrt(pow(x,2) + pow(y, 2));
  float z_prime = z - L1;
  float c3 = (pow(R,2) + pow(z_prime, 2) - pow(L2, 2) - pow(Lg, 2)) / (2.0 * L2 * Lg);
  float s3 = -sqrt(1.0 - pow(c3, 2));
  float theta3_rad = -atan2(s3, c3);


  if (c3 < -1.0 || c3 > 1.0){
    OK = false;
  }

  if (OK == false){
    Serial.print("The position is not reachable");
  } else {
    float K1 = L2 + Lg * c3;
    float K2 = Lg * s3;
  
    float beta = atan2(K2, K1);
    float theta2_rad = atan2(z_prime, R) - beta;
  
    int angle1 = round((theta1_rad * 180.0 / PI) + offset_theta1);
    int angle2 = round((theta2_rad * 180.0 / PI) + offset_theta2);
    int angle3 = -round((theta3_rad * 180.0 / PI) + offset_theta3);

    servo1.write(constrain(angle1, 0, 180));
    servo2.write(constrain(angle2, 0, 180));
    servo3.write(constrain(angle3, 0, 180));

    Serial.print("Angles: ");
    Serial.print(angle1); Serial.print(", ");
    Serial.print(angle2); Serial.print(", ");
    Serial.print(angle3); Serial.print(" | ");
  } 
}

/* The functions for trajectory planning */
/* 
  The parameters:
  p0[3]: an array represents the Cartesian coordinates of the starting point
  pf[3]: an array represents the Cartesian coordinates of the final point
*/

void moveStraight(float p0[3], float pf[3]){
  float tf = 5.0; // Total sampling time
  float dt = 0.1; // Time between each sample
  int i = 0; // Initialise the counter
  float last_samping_time = 0.0;
  int steps = tf/dt;
  while(i <= steps){
    current_time = millis();
    if(current_time - last_sampling_time >= dt*1000){
      // Define the constants of time to be used
      float t_i = i * dt;
      float tau = t_i/tf;

      // Substitute the time into the cubic polynomial, to get the current progress
      float s_i = 3 * pow(tau, 2) - 2 * pow(tau, 3);

      // Calculate the current Cartesian coordinates
      float x_i = p0[0] + (pf[0] - p0[0]) * s_i;
      float y_i = p0[1] + (pf[1] - p0[1]) * s_i;
      float z_i = p0[2] + (pf[2] - p0[2]) * s_i;

      // Apply the inverse kinematics to the three variables
      moveToPosition(x_i, y_i, z_i);

      // Update the counter
      i++;

      // Update the time of calculation
      last_sampling_time = current_time;
    }
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Attach servos
  baseServo.attach(2);
  shoulderServo.attach(3);
  elbowServo.attach(4);
  gripperServo.attach(11);
  delay(2000);
  baseServo.write(starting_angle1);
  shoulderServo.write(starting_angle2);
  elbowServo.write(starting_angle3);
  gripperServo.write(starting_angle_gripper);

}

void loop() {
  /* From initial point to point 1*/
  moveStraight(target_values[0], target_values[1]);
  /* Close gripper */
  gripperServo.write(90);
  /* From point 1 back to initial point */
  moveStraight(target_value[1], target_values[0]);
  /* From initial point to point 2 */
  moveStraight(target_value[0], target_value[2]);
  /* Open gripper */
  gripperServo.write(0);
  /* From point 2 to initial point */
  moveStraight(target_value[2], target_value[0]);
}
