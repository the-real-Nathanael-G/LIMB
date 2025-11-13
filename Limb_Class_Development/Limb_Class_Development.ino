#include <Matrix.h>
#include <Limb.h>

//Variables & Objects
const float L[3] = {7.0f, 15.0f, 10.0f}; // Link Lengths <------------------------------TODO: Change to correct lengths---------------------------------------------
const float R[3] = {0.0f, 0.64f, -2.05f}; // Joint angles when at rest (rads) <------------------------------TODO: Change to correct angles-------------------------
const float V[] = {0, 0, 0, 0, 0, L[0], PI/2, 0, 0, 0, 0, L[1], 0, 0, 0, L[2]};
const Matrix LegDHTable(4, 4, V);

void setup() {
  Serial.begin(115200);
  
  /*
  //MATRIX CLASS TESTBENCHES
  test_joins_fast();
  test_matmul_fast();
  test_copyRow_safe();
  test_determinant_inverse();
  */

  //LIMB CLASS TESTBENCHES
  Limb Leg = Limb(LegDHTable, L, R);
  Leg.runPrivateFunctionsTestbench();

}

void loop() {
  // put your main code here, to run repeatedly:

}
