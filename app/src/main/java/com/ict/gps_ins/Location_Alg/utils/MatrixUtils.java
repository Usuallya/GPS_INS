package com.ict.gps_ins.Location_Alg.utils;

import Jama.Matrix;

public class MatrixUtils {

    public static Matrix DiagMatrix(double[] eye) {
        int n = eye.length;
        Matrix result = new Matrix(n, n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i == j)
                    result.set(i, i, eye[i]);
                else
                    result.set(i, j, 0);
            }
        }
        return result;
    }

    public static Matrix EyeMatrix(int n) {
        Matrix result = new Matrix(n, n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i == j)
                    result.set(i, j, 1);
                else
                    result.set(i, j, 0);
            }
        }
        return result;
    }

}
