package com.ict.gps_ins.Location_Alg.utils;

import Jama.Matrix;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.tan;

public class Kalman {
    private final double deg2rad = 0.01745329237161968996669562749648;
    private final double rad2deg = 57.295780490442968321226628812406;
    private final double G0 = 9.8015;
    private final double wie = 7.292e-5;
    private final double e = 1 / 298.257;
    private final double Q_wg = pow(0.5 * deg2rad / 3600, 2);//陀螺的随机漂移为0.5度每小时
    private final double Q_wr = pow(0.1 * deg2rad / 3600, 2);
    private final double Q_wa = pow(0.5e-4 * G0, 2);//加速度计的随机偏差为0.5e-4*g
    private double[] PP0k = {
            pow(1/(36*57), 2), pow(1/(36*57), 2), pow(1/57, 2),
            pow(0.01, 2), pow(0.01, 2), pow(0.01, 2),
            0, 0, 1,
            pow(0.1 * deg2rad / 3600, 2), pow(0.1 * deg2rad / 3600, 2), pow(0.1 * deg2rad / 3600, 2),
            pow(0.04 * deg2rad / 3600, 2), pow(0.04 * deg2rad / 3600, 2), pow(0.04 * deg2rad / 3600, 2),
            pow(1e-4, 2), pow(1e-4, 2), pow(1e-4, 2)
    };
    private final double Tg = 300;//陀螺仪误差漂移相关时间
    private final double Ta = 1000;//加表误差漂移相关时间
    private final double Rlamt = 1e-5 * deg2rad / 60;//经纬度误差均方根，弧度制
    private final double Rl = 1e-5 * deg2rad / 60;
    private final double Rh = 9.591840841;//高度误差均方根，单位米
    private final double Rvx = 1e-7;//速度误差均方根，单位 米/秒
    private final double Rvy = 1e-7;
    private final double Rvz = 5e-9;
    private double[] Q_diag = {Q_wg, Q_wg, Q_wg, Q_wr, Q_wr, Q_wr, Q_wa, Q_wa, Q_wa};
    private double[] Rk = {Rlamt, Rl, Rh, Rvx, Rvy, Rvz};


    private Matrix Q;
    private Matrix R;
    private Matrix PP;
    private Matrix X;

    public Kalman() {
        PP = MatrixUtils.DiagMatrix(PP0k);
        Q = MatrixUtils.DiagMatrix(Q_diag);
        R = MatrixUtils.DiagMatrix(Rk);
        X = new Matrix(18, 1, 0);
    }

    public Matrix kalman_GPS_INS_pv(Matrix Dpv, double Ve, double Vn, double Vu,
                                    double L, double h,
                                    Matrix mahonyR, Matrix Fn, double tao,
                                    double Rm, double Rn) {

        double fe = Fn.get(0, 0);
        double fn = Fn.get(1, 0);
        double fu = Fn.get(2, 0);
        double C11 = mahonyR.get(0, 0);
        double C12 = mahonyR.get(0, 1);
        double C13 = mahonyR.get(0, 2);
        double C21 = mahonyR.get(1, 0);
        double C22 = mahonyR.get(1, 1);
        double C23 = mahonyR.get(1, 2);
        double C31 = mahonyR.get(2, 0);
        double C32 = mahonyR.get(2, 1);
        double C33 = mahonyR.get(2, 2);
        double cosL = cos(L);
        double sinL = sin(L);
        double tanL = tan(L);
        double secL = 1 / cos(L);
        double secL2 = 1 / pow(cos(L), 2);
        double Rnh = Rn + h;
        double Rmh = Rm + h;
        double Rnh2 = pow(Rn + h, 2);
        double Rmh2 = pow(Rm + h, 2);

        double[][] matrixF = {
                {               0               ,wie * sinL + Ve * tanL / Rnh,-(wie * cosL + Ve / Rnh),                0                  ,            -1 / Rmh            ,              0              ,                            0                            ,        0        ,            Vn / Rmh2            , C11 , C12 , C13 ,  C11  ,  C12  ,  C13  ,   0   ,   0   ,   0   },
                {-(wie * sinL + Ve * tanL / Rnh),             0              ,        -Vn / Rmh       ,            1 / Rnh                ,                0               ,              0              ,                        -wie * sinL                      ,        0        ,           -Ve / Rnh2            , C21 , C22 , C23 ,  C21  ,  C22  ,  C23  ,   0   ,   0   ,   0   },
                {     wie * cosL + Ve / Rnh     ,           Vn / Rmh         ,            0           ,            tanL / Rnh             ,                0               ,              0              ,                wie * cosL + Ve * secL2 / Rnh            ,        0        ,         -Ve * tanL / Rnh2       , C31 , C32 , C33 ,  C31  ,  C32  ,  C33  ,   0   ,   0   ,   0   },
                {               0               ,             -fu            ,            fn          ,        (Vn * tanL - Vu) / Rmh     ,2 * wie * sinL + Ve * tanL / Rnh,-(2 * wie * cosL + Ve / Rnh) ,2 * wie * (cosL * Vn + sinL * Vu) + Ve * Vn * secL2 / Rnh,        0        ,(Ve * Vu - Ve * Vn * tanL) / Rnh2,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,  C11  ,  C12  ,  C13  },
                {               fu              ,             0              ,            -fe         ,-2 * (wie * sinL + Ve * tanL / Rnh),            -Vu / Rmh           ,         -Vn / Rmh           ,        -(2 * wie * cosL * Ve + Ve * Ve * secL2 / Rnh)   ,        0        ,(Ve * Ve * tanL + Vn * Vu) / Rnh2,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,  C21  ,  C22  ,  C23  },
                {               -fn             ,             fe             ,            0           ,    2 * (wie * cosL + Ve / Rnh)    ,            2 * Vn / Rmh        ,              0              ,                    -2 * wie * sinL * Ve                 ,        0        ,   -(Ve * Ve + Vn * Vn) / Rnh2   ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,  C31  ,  C32  ,  C33  },
                {               0               ,             0              ,            0           ,                0                  ,                1 / Rmh         ,              0              ,                            0                            ,        0        ,           -Vn / Rmh2            ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,            secL / Rnh             ,                0               ,              0              ,                    Ve * secL * tanL / Rnh               ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              1              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,-1 / Tg,   0   ,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,-1 / Tg,   0   ,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,-1 / Tg,   0   ,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,-1 / Ta,   0   ,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,-1 / Ta,   0   },
                {               0               ,             0              ,            0           ,                0                  ,                0               ,              0              ,                            0                            ,        0        ,               0                 ,  0  ,  0  ,  0  ,   0   ,   0   ,   0   ,   0   ,   0   ,-1 / Ta},
        };//18X18转移矩阵
        Matrix F = new Matrix(matrixF);

        double[][] matrixG = {
                { C11 , C12 , C13 ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                { C21 , C22 , C23 ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                { C31 , C32 , C33 ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  0  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  1  ,  0  },
                {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  1  },
        };//18X9连续系统输入矩阵
        Matrix G = new Matrix(matrixG);

        double[][] matrixH = {
                {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        };//6X18连续系统量测矩阵
        Matrix H = new Matrix(matrixH);

        Matrix eye18 = MatrixUtils.EyeMatrix(18);
        Matrix A = eye18.plus(F.times(tao));//18X18
        Matrix B = eye18.plus(F.times(tao).times(0.5)).times(G.times(tao));//18X9

        //KF Start
        Matrix P = A.times(PP).times(A.transpose()).plus(B.times(Q).times(B.transpose()));//18X18
        Matrix K = P.times(H.transpose()).times(H.times(P).times(H.transpose()).plus(R).inverse());//
        PP = eye18.minus(K.times(H)).times(P);
        PP = PP.plus(PP.transpose()).times(0.5);
        Matrix Z = Dpv;
        Matrix XX = A.times(X).plus(K.times(Z.minus(H.times(A).times(X))));
        X = XX;
        //KF start
//        X = A.times(X);//X = A*X
//        Matrix P = A.times(PP).times(A.transpose()).plus(B.times(Q).times(B.transpose()));//P = A*PP*AT + B*Q*BT
//        Matrix Y = H.times(P).times(H.transpose()).plus(R);//Y = H*P*HT + R
//        Matrix K = P.times(H.transpose()).times(Y.inverse());//K = P*HT*Y-1
//        PP = eye18.minus(K.times(H)).times(P);//PP = (eye15-K*H)*P
//        Matrix Z = Dpv;
//        Matrix W = Z.minus(H.times(X));//W = Z - H*X
//        Matrix XX = X.plus(K.times(W));//XX = X + K*W
        return XX;
    }

}
