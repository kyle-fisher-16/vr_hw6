#include "PoseMath.h"

/**
 * TODO: see header file for documentation
 */
 /**
 * convert RAW clock ticks to 2D positions
 * use the variable CLOCKS_PER_SECOND defined above
 * @param [in] clockTicks - raw ticks of timing values in x and y
 *  for each of the 4 photodiodes
 * @param [out] pos2D positions of measurements on plane at
 *   unit distance
 */ 
 // clock ticks is ordered x0, y0, x1, y1, etc...
void convertTicksTo2DPositions(uint32_t clockTicks[8], double pos2D[8])
{
  double delta_t, alpha;
  for (unsigned int i = 0; i < 8; i++) {
    delta_t = (double)clockTicks[i] / CLOCKS_PER_SECOND;

    // find alpha_h or alpha_v depending on whether this is x or y 
    if (i%2 == 0) { // x coordinate
      alpha = - delta_t * 60 * 360 + 90;
    }
    else { // y coordinate
      alpha = delta_t * 60 * 360 - 90;
    }
    
    pos2D[i] = tan(alpha * PI / 180.0);
  }
  //use variable CLOCKS_PER_SECOND defined in PoseMath.h
  //for number of clock ticks a second


}

/**
 * form matrix A, that maps sensor positions, b, to homography parameters, h:
 *  b = Ah
 * See course notes for derviation
 * @param [in]  pos2D  - lighthouse measurements of 2D photodiode projections
    on plane at unit distance away.
 *  this is a 8-element array of doubleing point values with order:
 *  [sensor0x, sensor0y, ... sensor3x, sensor3y]
 * @param [in] posRef - actual 2D positions of photodiodes. this
 *  is set based on the layout of the board. units is in mm, order is
 *  [sensor0x, sensor0y, ... sensor3x, sensor3y]
 * @param [out] AOut - 8x8 output matrix. A[i][j] refers to A_{i,j}
 */
void formA(double pos2D[8], double posRef[8], double Aout[8][8]) {

  // i keeps track of the photodiode # (0, 1, 2, 3)
  unsigned int i = 0;
<<<<<<< HEAD
  double x_i, y_i, xn_i, yn_i; //photodiode postions and normalized positions
=======
  double x_i, y_i, xn_i, yn_i;
>>>>>>> a41f255368167b3713600365053d169ca0c88f80

  // row keeps track of which row we are writing. 
  for (unsigned int row = 0; row < 8; row += 2) {
    x_i = posRef[i*2]; // sensor x
    y_i = posRef[i*2 + 1]; // sensor y
    xn_i = pos2D[i*2]; // pos2d x
    yn_i = pos2D[i*2 + 1]; // pos2d x

    // did it this way for readability. make two rows at a time!
    double x_row[8] = { x_i,  y_i,  1.0,  0.0,  0.0,  0.0, -x_i*xn_i, -y_i*xn_i };
    double y_row[8] = { 0.0,  0.0,  0.0,  x_i,  y_i,  1.0, -x_i*yn_i, -y_i*yn_i };
    // copy the rows over to the output array
    for (int j=0; j < 8; j++) {
      Aout[row][j]   = x_row[j];
      Aout[row+1][j] = y_row[j];
    }

    // now let's move to the next photodiode
    i += 1;
  }
}



 
 // USE THIS
// void MatrixMath::Multiply(double* A, double* B, int m, int p, int n, double* C)
//    // A = input matrix (m x p)
//    // B = input matrix (p x n)
//    // m = number of rows in A
//    // p = number of columns in A = number of rows in B
//    // n = number of columns in B
//    // C = output matrix = A*B (m x n)
 
bool solveForH(double A[8][8], double b[8], double hOut[8]) {
  //use Matrix Math library for matrix operations
  if(!Matrix.Invert((double*)A, 8)) return false;
  //else{Matrix.Invert((double*)A, 8);}

  // treat b as a 8x1 matrix
  double b_mat[8][1];
  for (unsigned int i = 0; i < 8; i++) {
    b_mat[i][0] = b[i];
  }
  
  Matrix.Multiply((double*)A, (double*)b_mat, 8, 8, 1, (double*)hOut);
  
  return true;

}


double l2norm(double x, double y, double z) {
  return sqrt(x*x + y*y + z*z);
}

/**
 * TODO: see header file for documentation
 */
void getRtFromH(double h[8], double ROut[3][3], double pos3DOut[3]) {

  // scale according to the norms of the first two columns
  double s = 2 / (l2norm(h[0], h[3], h[6]) + l2norm(h[1], h[4], h[7]));
  for (unsigned int i = 0; i < 8; i++) {
    h[i] *= s;
  }

  // column 1
  double col1_norm = l2norm(h[0], h[3], h[6]);
  ROut[0][0] = h[0] / col1_norm;
  ROut[1][0] = h[3] / col1_norm;
  ROut[2][0] = -h[6] / col1_norm;
  

  // column 2
  /*double col2_norm = l2norm(h[1], h[4], h[7]);
  ROut[0][1] = h[1] / col2_norm;
  ROut[1][1] = h[4] / col2_norm;
  ROut[2][1] = -h[7] / col2_norm;*/
  // need to enforce orthogonality with column 1
  double r11 = ROut[0][0];
  double r21 = ROut[1][0];
  double r31 = ROut[2][0];
  
  ROut[0][1] = h[1] - (r11*(r11*h[1] + r21*h[4] - r31*h[7]));
  ROut[1][1] = h[4] - (r21*(r11*h[1] + r21*h[4] - r31*h[7]));
  ROut[2][1] = h[7] - (r31*(r11*h[1] + r21*h[4] - r31*h[7]));
  //normalize!
  double col2_norm = l2norm(ROut[0][1], ROut[1][1], ROut[2][1]);
  ROut[0][1] = ROut[0][1] / col2_norm;
  ROut[1][1] = ROut[1][1] / col2_norm;
  ROut[2][1] = ROut[2][1] / col2_norm;

  // column 3 (using cross product)
  ROut[0][2] = ROut[1][0] * ROut[2][1] - ROut[2][0] * ROut[1][1];
  ROut[1][2] = ROut[2][0] * ROut[0][1] - ROut[0][0] * ROut[2][1];
  ROut[2][2] = ROut[0][0] * ROut[1][1] - ROut[1][0] * ROut[0][1];

  // position
  pos3DOut[0] = h[2];
  pos3DOut[1] = h[5];
  pos3DOut[2] = -s;
  
}



/**
 * TODO: see header file for documentation
 */
Quaternion getQuaternionFromRotationMatrix(double R[3][3]) {
  double qw = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2;
  double qx = (R[2][1] - R[1][2]) / (4.0 * qw);
  double qy = (R[0][2] - R[2][0]) / (4.0 * qw);
  double qz = (R[1][0] - R[0][1]) / (4.0 * qw);

  return Quaternion(qw, qx, qy, qz).normalize();

}
