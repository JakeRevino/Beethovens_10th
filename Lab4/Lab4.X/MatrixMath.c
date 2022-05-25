/* This file was made by a former CSE13 student Ashwin Chakra, who was kind 
 * enough to allow me to use his old CSE13 MatrixMath code.
 * I never had to take CSE13 so I did not have these files, until now.  
 */

#include <stdio.h>
#include <stdlib.h>
#include "MatrixMath.h"

// Helper function

float MatrixDeterminant2(float mat[2][2]) {
    return mat[0][0] * mat [1][1] - mat[1][0] * mat[0][1];
}

int MatrixEquals2(float mat1[2][2], float mat2[2][2]) {
    int i, j;
    for (i = 0; i < (DIM - 1); i++) {
        for (j = 0; j < (DIM - 1); j++) {
            if (abs(mat1[i][j] - mat2[i][j]) > FP_DELTA)
                return 0;
        }
    }
    return 1;
}

/*
 * 
 */
void MatrixPrint(float mat[3][3]) {
    int i, j;
    printf("\r_________________________\n\r");
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            printf("%6.2f | ", mat[i][j]);
        }
        printf("\n\r-------------------------\n\r");
    }
    printf("\r_________________________\n\n\n\r");
}

int MatrixEquals(float mat1[3][3], float mat2[3][3]) {
    int i, j;
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            if (abs(mat1[i][j] - mat2[i][j]) > FP_DELTA)
                return 0;
        }
    }
    return 1;
}

void MatrixAdd(float mat1[3][3], float mat2[3][3], float result[3][3]) {
    int i, j;
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    }
}

void MatrixAdd31(float mat1[3][1], float mat2[3][1], float result[3][1]) {
    int i;
    for (i = 0; i < DIM; i++) {
            result[i][0] = mat1[i][0] + mat2[i][0];     
    }
}

void MatrixSubtract(float mat1[3][3], float mat2[3][3], float result[3][3]) {
    int i, j;
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            result[i][j] = mat1[i][j] - mat2[i][j];
        }
    }
}

void MatrixMultiply(float mat1[3][3], float mat2[3][3], float result[3][3]) {
    int i, j, k;
    // need to reset all the values of result to zero to avoid using values from previous test
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            result[i][j] = 0;
        }
    }
    // need to use three for loops in order to use row major for mat1 and column major for mat2
    // as well as repeated uses of the correct row of mat1
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            for (k = 0; k < DIM; k++) {
                result[i][j] += +mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void MatrixMultiply13(float mat1[3][3], float mat2[3], float result[3]) {
    // need to reset all the values of result to zero to avoid using values from previous test
    result[0] = 0;
    result[1] = 0;
    result[2] = 0;

    // pretend I did something clever with for loops and such
    result[0] = mat1[0][0] * mat2[0] + mat1[0][1] * mat2[1] + mat1[0][2] * mat2[2];
    result[1] = mat1[1][0] * mat2[0] + mat1[1][1] * mat2[1] + mat1[1][2] * mat2[2];
    result[2] = mat1[2][0] * mat2[0] + mat1[2][1] * mat2[1] + mat1[2][2] * mat2[2];
}

void MatrixMultiply31(float mat1[3][3], float mat2[3][1], float result[3][1]) {
    // need to reset all the values of result to zero to avoid using values from previous test
    result[0][0] = 0;
    result[1][0] = 0;
    result[2][0] = 0;

    result[0][0] = mat1[0][0] * mat2[0][0] + mat1[0][1] * mat2[1][0] + mat1[0][2] * mat2[2][0];
    result[1][0] = mat1[1][0] * mat2[0][0] + mat1[1][1] * mat2[1][0] + mat1[1][2] * mat2[2][0];
    result[2][0] = mat1[2][0] * mat2[0][0] + mat1[2][1] * mat2[1][0] + mat1[2][2] * mat2[2][0];
}

void MatrixScalarAdd(float x, float mat[3][3], float result[3][3]) {
    int i, j;
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            result[i][j] = mat[i][j] + x;
        }
    }
}

void MatrixScalarMultiply(float x, float mat[3][3], float result[3][3]) {
    int i, j;
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            result[i][j] = mat[i][j] * x;
        }
    }
}

void MatrixScalarMultiply31(float x, float mat[3][1], float result[3][1]) {
    int i, j;
    for (i = 0; i < DIM; i++) {
        result[i][0] = mat[i][0] * x;
    }
}

float MatrixTrace(float mat[3][3]) {
    int i, j;
    j = 0;
    for (i = 0; i < DIM; i++) {
        j += mat[i][i];
    }
    return j;
}

void MatrixTranspose(float mat[3][3], float result[3][3]) {
    int i, j;
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            result[j][i] = mat[i][j];
        }
    }
}

void MatrixSubmatrix(int i, int j, float mat[3][3], float result[2][2]) {
    int a, b, k;
    k = 0;
    float temp[(DIM - 1)*(DIM - 1)] = {};
    // this for loop skips over row i and column j with continue statements and puts everything
    // else in order into a temp array
    for (a = 0; a < DIM; a++) {
        if (a == i) {
            continue;
        }
        for (b = 0; b < DIM; b++) {
            if (b == j) {
                continue;
            }
            temp[k] = mat[a][b];
            k++;
        }
    }
    // this for loop places the values in the temp array into the 2x2 matrix in the proper places
    k = 0;
    for (a = 0; a < (DIM - 1); a++) {
        for (b = 0; b < (DIM - 1); b++) {
            result[a][b] = temp[k];
            k++;
        }
    }
}

float MatrixDeterminant(float mat[3][3]) {
    int i, result, ans;
    float output[2][2] = {
        {},
        {}
    };
    // the submatrix is created using a previous function that had already been tested and the
    // determinant of the 2x2 matrix is using a helper function that has also already been tested
    for (i = 0; i < DIM; i++) {
        MatrixSubmatrix(0, i, mat, output);
        ans = mat[0][i] * MatrixDeterminant2(output);
        if (i % 2 == 1) {
            ans = ans * -1;
        }
        result += ans;
    }
    return result;
}

void MatrixInverse(float mat[3][3], float result[3][3]) {
    int i, j, detA;
    float output[2][2] = {
        {},
        {}
    };
    float cofactor[3][3] = {
        {},
        {},
        {}
    };
    float transpCof[3][3] = {
        {},
        {},
        {}
    };
    // this function also uses many previous functions that have already been tested for errors
    for (i = 0; i < DIM; i++) {
        for (j = 0; j < DIM; j++) {
            if (((i + j) % 2) == 0) {
                MatrixSubmatrix(i, j, mat, output);
                cofactor[i][j] = MatrixDeterminant2(output);
            }
            MatrixSubmatrix(i, j, mat, output);
            cofactor[i][j] = -1 * MatrixDeterminant2(output);
        }
    }
    MatrixTranspose(cofactor, transpCof);
    detA = MatrixDeterminant(mat);
    MatrixScalarMultiply(1 / detA, transpCof, result);
}
