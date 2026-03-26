#pragma once


/*
        calculates the determinant of a 2x2 matrix 
*/
float determinant_2x2(float arr[2][2]);

/*
        calculates the determinant of a 3x3 matrix
*/
float determinant_3x3(float arr[3][3]);

/*
        multiply the matrix by some scalar k
*/
void scalar_multiplication_3x3(float res[3][3], float arr[3][3], float k); 

/*
        multiply a 3x3 matrix by a 3x1 vector
*/
void matrix_vector_multiplication_3x3(float res[3], float arr[3][3], float vec[3]);

void matrix_addition_3x3(float res[3][3], float A[3][3], float B[3][3]);

void matrix_subtraction_3x3(float res[3][3], float A[3][3], float B[3][3]);

void vector_addition_3(float res[3], float a[3], float b[3]);

void vector_subtraction_3(float res[3], float a[3], float b[3]);

void scalar_multiplication_3(float res[3], float vec[3], float k);

void transpose_3x3(float res[3][3], float arr[3][3]);

void matrix_multiplication_3x3(float res[3][3], float A[3][3], float B[3][3]);

void copy_3x3(float res[3][3], float arr[3][3]);

void symmetrize_3x3(float arr[3][3]);

void outer_product_3x1(float res[3][3], float a[3], float b[3]);

void copy_3(float res[3], float arr[3]);

void identity_3x3(float res[3][3]);

float dot_product_3(float a[3], float b[3]);