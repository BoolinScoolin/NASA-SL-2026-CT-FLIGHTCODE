#include "matrix_comp.h"

float determinant_2x2(float arr[2][2])
{
        return arr[0][0] * arr[1][1] - arr[0][1] * arr[1][0]; 
}

float determinant_3x3(float arr[3][3])
{
        float cofactor_0 = arr[1][1] * arr[2][2] - arr[1][2] * arr[2][1];
        float cofactor_1 = arr[1][0] * arr[2][2] - arr[1][2] * arr[2][0];
        float cofactor_2 = arr[1][0] * arr[2][1] - arr[1][1] * arr[2][0];
        return arr[0][0] * cofactor_0 - arr[0][1] * cofactor_1 + arr[0][2] * cofactor_2;
}

void scalar_multiplication_3x3(float res[3][3], float arr[3][3], float k)
{
        for (int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        res[i][j] = k * arr[i][j];
                }
        }
}

void matrix_vector_multiplication_3x3(float res[3], float arr[3][3], float vec[3])
{
        for(int i = 0; i < 3; i++)
        {
                res[i] = 0;
                for(int j = 0; j < 3; j++)
                {
                        res[i] += arr[i][j] * vec[j];
                }
        }
}

void matrix_addition_3x3(float res[3][3], float A[3][3], float B[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = A[i][j] + B[i][j];
        }
    }
}

void matrix_subtraction_3x3(float res[3][3], float A[3][3], float B[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = A[i][j] - B[i][j];
        }
    }
}

void vector_addition_3(float res[3], float a[3], float b[3])
{
    for (int i = 0; i < 3; i++)
    {
        res[i] = a[i] + b[i];
    }
}


void vector_subtraction_3(float res[3], float a[3], float b[3])
{
    for (int i = 0; i < 3; i++)
    {
        res[i] = a[i] - b[i];
    }
}

void scalar_multiplication_3(float res[3], float vec[3], float k)
{
    for (int i = 0; i < 3; i++)
    {
        res[i] = k * vec[i];
    }
}

void transpose_3x3(float res[3][3], float arr[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = arr[j][i];
        }
    }
}

void matrix_multiplication_3x3(float res[3][3], float A[3][3], float B[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void copy_3x3(float res[3][3], float arr[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = arr[i][j];
        }
    }
}

void symmetrize_3x3(float arr[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            float avg = 0.5f * (arr[i][j] + arr[j][i]);
            arr[i][j] = avg;
            arr[j][i] = avg;
        }
    }
}

void outer_product_3x1(float res[3][3], float a[3], float b[3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = a[i] * b[j];
        }
    }
}

void copy_3(float res[3], float arr[3])
{
    for (int i = 0; i < 3; i++)
    {
        res[i] = arr[i];
    }
}

void identity_3x3(float res[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            res[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

float dot_product_3(float a[3], float b[3])
{
    float res = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        res += a[i] * b[i];
    }
    return res;
}
