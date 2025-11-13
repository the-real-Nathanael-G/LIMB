/*
Nathanael Georges, 29/10/2025.
Class for describing robotic limbs and handling motion calculations.

NOTES:
-Currently fixed to only handle 3DoF Limbs.

TODO:
-calculateMotion()
-calcFK()
-calcIK_Geometric()
-restPos()
-Make the constructor take in a DH table as input, then assign no. of links dynamically - this will allow this class to be used for any given limb.
-Add more complex IK solvers.
-Add Dynamics
*/

#include <Matrix.h>

class Limb {
private:

  // Variables
  const Matrix DHTable;
  const int DoF;
  float stepSize; // Stride length
  float direction; // Direction of travel (rads)
  const std::array<float, 3> L; // Link lengths
  const std::array<float, 3> R; // Limb joints at rest
  Matrix tAngles; // Theta values in radians
  Matrix PWM; // Theta values as PWM-ready signals

  // Calculates the Homogeneous Transformation Matrix for a single link given DH parameters (Classical Method)
  Matrix calcHT(float theta, float d, float alpha, float r) {
    Matrix T(4, 4);

    const float ct = cosf(theta);
    const float st = sinf(theta);
    const float ca = cosf(alpha);
    const float sa = sinf(alpha);

    T(0, 0) = ct;
    T(0, 1) = -st * ca;
    T(0, 2) = st * sa;
    T(0, 3) = r * ct;

    T(1, 0) = st;
    T(1, 1) = ct * ca;
    T(1, 2) = -ct * sa;
    T(1, 3) = r * st;

    T(2, 0) = 0;
    T(2, 1) = sa;
    T(2, 2) = ca;
    T(2, 3) = d;

    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;
    return T;
  }

  // Calculates the overall homogeneous transformation matrix from the Origin to the End Effector at timestep t
  Matrix calcLimbHT(int t) {
    // Start with 'blank' matrix that does not affect following calculations
    Matrix T0_4(4, 4);
    T0_4.fill(1.0f);

    for (int i = 0; i < DoF; i++){
      // Calculate the homogeneous transformation for each link
      Matrix LinkI = calcHT(tAngles(i, t) + DHTable(i, 0), DHTable(i, 1), DHTable(i, 2), DHTable(i, 3));
      
      // Append link to previous links
      T0_4 = T0_4 * LinkI;

      // Test to show new HT Matrix with each link
      Serial.println("");
      Serial.print("At Link ");
      Serial.print(i);
      Serial.println("");
      T0_4.print();
    }

    return T0_4;
  }

  // Calculates overall HT Matrix up to specified link 'x' at timestep t
  Matrix calcLimbHT(int t, int x) {
    // Start with 'blank' matrix that does not affect following calculations
    Matrix T0_x(4, 4);
    T0_x.fill(1.0f);

    // Calculate the homogeneous transformation for each link
    for (int i = 0; i < x; i++){
      Matrix LinkI = calcHT(tAngles(i, t) + DHTable(i, 0), DHTable(i, 1), DHTable(i, 2), DHTable(i, 3));
      
      // Append link to previous links
      T0_x = T0_x * LinkI;
      Serial.println("");
      Serial.print("At Link ");
      Serial.print(i);
      Serial.println("");
      T0_x.print();
    }

    return T0_x;
  }

  //Calculate Forward Kinematics and return 3xt matrix with xyz coords of end effector at each timestep
  Matrix calcFK(){
    //TODO
  }

public:
  // Constructor for 4-link 3DoF Hexapod leg
  Limb(const Matrix& TemplateTable, const float* LinkLengths, const float* RestAngles) : DHTable(TemplateTable),
          DoF(TemplateTable.getRows()),
          L{LinkLengths[0], LinkLengths[1], LinkLengths[2]},
          R{RestAngles[0], RestAngles[1], RestAngles[2]} {

    // TestBench
    Serial.print("R = ");
    for(int i = 0; i<3; i++){
      Serial.print(R[i]);
      Serial.print(", ");
    }
    Serial.println("");
    Serial.print("L = ");
    for(int i = 0; i<3; i++){
      Serial.print(L[i]);
      Serial.print(", ");
    }
    Serial.println("");
    Serial.println("DH Table:");
    DHTable.print();
  }

  void runPrivateFunctionsTestbench(){
    // Single HT Matrix Test
    Serial.println("");
    Serial.print("Single HT Matrix Calc Test: ");
    Matrix TransformTestA = calcHT(PI/2, 19, PI/4, 6);
    Serial.println("");
    TransformTestA.print();

    // Overall HT Matrix Test
    Serial.println("");
    Serial.print("Overall HT Matrix Test: ");
    tAngles.reSize(3, 1);
    tAngles(0, 0) = PI/2;
    tAngles(1, 0) = PI/4;
    tAngles(2, 0) = PI/3;
    Matrix TransformTestB = calcLimbHT(0);
    Serial.println("");
    //TransformTestB.print();

    // 0 to x HT Matrix Test
    Serial.println("");
    Serial.print("Overall HT Matrix Test: ");
    tAngles(0, 0) = PI/6;
    tAngles(1, 0) = PI/2;
    tAngles(2, 0) = PI/5;
    Matrix TransformTestC = calcLimbHT(0, 1);
    Serial.println("");
    //TransformTestC.print();

  }

  // Destructor
};
