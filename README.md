# 6_DOF_ROBOTIC_ARM
A 6 Degrees of Freedom (6-DOF) robotic arm is a highly versatile and sophisticated robotic system widely used in industrial automation, research, and advanced manufacturing. The term "6-DOF" refers to the six independent axes of motion, allowing the arm to achieve full spatial manipulation: three translational movements (X, Y, and Z directions) and three rotational movements (roll, pitch, and yaw).

Key Features:
1. Flexibility: The 6-DOF design enables the robotic arm to replicate the dexterity of a human arm, making it capable of reaching, rotating, and orienting in complex 3D spaces.
2. Precision and Accuracy: High-resolution sensors and actuators provide unparalleled accuracy in handling delicate tasks.
3. Applications: Commonly used for tasks such as welding, pick-and-place operations, assembly, material handling, and medical surgeries.
Programmability: These robotic arms can be programmed with advanced algorithms for tasks requiring repeatability and adaptability.
Benefits:
1. Efficiency: Increases productivity and reduces operational costs.
2. Scalability: Can be adapted for both small-scale and large-scale operations.
3. Safety: Minimizes human intervention in hazardous environments.

In the coding part first we calculated all the orentation by the use of coordinates then write the code of using python language then we convert the code into an Arduino IDE code,
The Arduino Code is given below:

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

//using namespace BLA;



// Define constants
const float d1 = 1.0;
const float d2 = 1.0;
const float d3 = 5.0;
const float d4 = 5.0;
const float d5 = 0.2;
const float d6 = 0.0;



float cosineLaw(float a, float b, float c) {
    return acos((a * a + b * b - c * c) / (2 * a * b));
}




// Function to calculate the rotation matrix
void ROTATION_MATRIX(double roll, double pitch, double yaw, BLA::Matrix<3,3> &R0_6Y) {


        // Initial rotation matrix
      BLA::Matrix<3, 3>R0_6_initial = {0, 0, 1,0, -1, 0,1, 0, 0};
      // Rotation matrix for roll, pitch, and yaw
      BLA::Matrix<3,3> R0_6X = {
        cos(pitch) * cos(roll),
        sin(pitch) * sin(yaw) - sin(roll) * cos(pitch) * cos(yaw),
        sin(pitch) * cos(yaw) + sin(roll) * sin(yaw) * cos(pitch),

        sin(roll),
        cos(roll) * cos(yaw),
        -sin(yaw) * cos(roll),

        -sin(pitch) * cos(roll),
        sin(pitch) * sin(roll) * cos(yaw) + sin(yaw) * cos(pitch),
        -sin(pitch) * sin(roll) * sin(yaw) + cos(pitch) * cos(yaw)
      };

      // Compute the final rotation matrix R0_6 = R0_6_initial * R_6_rotation
      

      R0_6Y =  R0_6_initial * R0_6X;
      
    //   for(int i=0;i<3;i++){
    //   for(int j=0;j<3;j++){
    //     float x = R0_6Y(i,j);
    //     Serial.print(x);
    //     Serial.print("  ");
    //   }
    //    Serial.println("");
    // }


      
};


//gripper coordinate using function
void calculateGripperPosition(float finalX, float finalY, float finalZ, float d5, BLA::Matrix<3,3,float> &R0_6, float &Xgripper, float &Ygripper, float &Zgripper) {
    // Calculate the gripper position
    Xgripper = finalX - d5 * R0_6(0,2);
    Ygripper = finalY - d5 * R0_6(1,2);
    Zgripper = finalZ - d5 * R0_6(2,2);
};



void calculateInverseKinematics(float x , float y ,float z ,float &theta1 , float &theta2 , float &theta3) {
    // Calculate r and s
    float r = sqrt(x * x + y * y);
    float s = z - d1;

    // Calculate r2
    float r2 = sqrt(pow(r - d2, 2) + s * s);

    // Calculate theta1, theta2, theta3
    theta1 = atan2(y, x);
    float phi3 = cosineLaw(d3, d4, r2);
    theta3 = phi3 - PI;
    float phi1 = atan(s / (r - d2));
    float phi2 = cosineLaw(d3, r2, d4);
    theta2 = phi1 + phi2;

    Serial.print("Theta 1: ");
    Serial.println(theta1*180/M_PI);
    Serial.print("Theta 2: ");
    Serial.println(theta2*180/M_PI);
    Serial.print("Theta 3: ");
    Serial.println(theta3*180/M_PI);
};



// define roation matrix (R0_1,R1_2,R2_3)for R0_3 MATRIX 

// Define matrix types using BLA library
//typedef BLA::Matrix<3, 3, float> myMatrix;



    

void INVERSE_KINEMATICS_LAST_3(float theta1, float theta2, float theta3, BLA::Matrix<3,3,float> &R0_6) {
    // Compute individual rotation matrices

     
    // Rotation matrix around the z-axis FRAME 1 
    BLA::Matrix<3 ,3 > myMatrix_M1 = {1, 0, 0,0, 0, -1,0, 1, 0};
    BLA::Matrix<3,3 > myMatrix_Rz1 = {cos(theta1), -sin(theta1), 0,sin(theta1), cos(theta1), 0, 0, 0, 1};
    BLA::Matrix<3,3 > one =  myMatrix_Rz1 * myMatrix_M1;
    
    // Rotation matrix around the z-axis
    BLA::Matrix<3 ,3 > myMatrix_M2 = {1, 0, 0,0, 1, 0,0, 0, 1};
    BLA::Matrix<3,3 > myMatrix_Rz2 = {cos(theta2), -sin(theta2), 0,sin(theta2), cos(theta2), 0,0, 0, 1};
    BLA::Matrix<3,3 > two =  myMatrix_Rz2 * myMatrix_M2;

    BLA::Matrix<3 ,3 > myMatrix_M3 = {0, 0, 1,1, 0, 0,0, 1, 0};
    BLA::Matrix<3,3 > myMatrix_Rz3 = {cos(theta3), -sin(theta3), 0,sin(theta3), cos(theta3), 0,0, 0, 1};
    BLA::Matrix<3,3 > third =  myMatrix_Rz3 * myMatrix_M2;

    

    
    // Calculate R0_3
    BLA::Matrix<3,3> R0_3 = one * (two * third);  
     
    BLA::Matrix<3, 3> R0_3_new;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // Shift columns: column 1 -> column 2, column 2 -> column 3, column 3 -> column 1
            R0_3_new(i, (j + 2) % 3) = R0_3(i, j);
        }
    }




   // Serial.println(R0_3(0,0));
    // for(int i=0;i<3;i++){
    //   for(int j=0;j<3;j++){
    //     float x = R0_3_new(i,j);
    //     Serial.print(x);
    //     Serial.print("  ");
    //   }
    //    Serial.println("");
    // }

    // Compute the inverse of R0_3                      
    BLA::Matrix<3,3 > R0_3inv = Inverse(R0_3_new);


    // Serial.print("prnce");
    // Serial.println(R0_3inv(0,2));
    // for(int i=0;i<3;i++){
    //   for(int j=0;j<3;j++){
    //     float x = R0_3inv(i,j);
    //     Serial.print(x);
    //     Serial.print("  ");
    //   }
    //    Serial.println("");
    // }
                                                           
    // BLA::Matrix<3, 3> R0_6M;
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j < 3; ++j) {
    //         R0_6M(i, j) = R0_6[i][j];
    //     } 
    // }

    // for(int i=0;i<3;i++){
    //   for(int j=0;j<3;j++){
    //     float x = R0_6(i,j);
    //     Serial.print(x);
    //     Serial.print("  ");
    //   }
    //    Serial.println("");
    // }

    // Calculate R3_6
    BLA::Matrix<3,3 > R3_6 = R0_3inv * R0_6;
    
    

    // Calculate theta5
    float theta5 = acos(R3_6(2, 2));

    // Calculate theta6
    float theta6 = asin(R3_6(2, 1) / sin(theta5));

    // Calculate theta4
    float theta4 = asin(R3_6(1, 2) / sin(theta5));

    // Print results
    Serial.print("Theta 4 = ");
    Serial.println(degrees(theta4));
    Serial.print("Theta 5 = ");
    Serial.println(degrees(theta5));
    Serial.print("Theta 6 = ");
    Serial.println(degrees(theta6));
}




void setup() {
        
        Serial.begin(9600);

        // Example inputs
        float finalX = 5.0;
        float finalY = 1.0;
        float finalZ = 1.0;
        float d5 = 0.2;

        // Example R0_6 matrix (3x3)
        BLA::Matrix<3,3> R0_6;

        double roll = 25.0*M_PI/180;
        double pitch = 30.0*M_PI/180;
        double yaw = 45.0*M_PI/180;

        // Variables to hold the results
        float Xgripper, Ygripper, Zgripper;
        float theta1 ,theta2 , theta3 ;

        // call function for rotation matrix
        ROTATION_MATRIX(roll,  pitch,  yaw,  R0_6);

      

      for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        float x = R0_6(i,j);
        Serial.print(x);
        Serial.print("  ");
      }
       Serial.println("");
    }
        // Call the function
        calculateGripperPosition(finalX, finalY, finalZ, d5, R0_6, Xgripper, Ygripper, Zgripper);

        // call function for first three angle
        calculateInverseKinematics(Xgripper, Ygripper, Zgripper, theta1 ,theta2 ,theta3);

        // call function for last three angle
        INVERSE_KINEMATICS_LAST_3(theta1, theta2, theta3, R0_6);
}
void loop (){

}
