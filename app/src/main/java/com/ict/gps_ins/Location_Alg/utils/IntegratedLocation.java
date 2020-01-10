package com.ict.gps_ins.Location_Alg.utils;

import com.ict.gps_ins.Location_Alg.Tools.Smooth;
import com.ict.gps_ins.Location_Alg.model.Datas;
import com.ict.gps_ins.Location_Alg.model.NMEA_GPS_INFO;

import java.math.BigDecimal;


import Jama.Matrix;

import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class IntegratedLocation {
    private Kalman kf;
    private Smooth smooth;

    public IntegratedLocation() {
        this.init();
        samplePeriod = 0.2;
    }

    public IntegratedLocation(double samplePeriod) {
        this.init();
        this.samplePeriod = samplePeriod;
    }

    private void init() {
        kf = new Kalman();
        smooth = new Smooth();
    }


    private final double rad2deg = 57.295780490442968321226628812406;
    private final double deg2rad = 0.01745329237161968996669562749648;
    //Re长半轴 r短半轴 f椭-球扁率 e椭球偏心率 wie地球自转角速率
    private final double earthRe = 6378137, earthr = 6356752.3142, earthf = 1 / 298.257, earthe = 0.0818, earthwie = 7.292e-5;
    //加速度零偏
    private double accCalStc_X = -0.09092;
    private double accCalStc_Y = 0.081208;
    private double accCalStc_Z = 0.015632;
    private double gyoCalStc_X = 0.00122173049021512;
    private double gyoCalStc_Y = 0.00122173049021512;
    private double gyoCalStc_Z = 0.00122173049021512;


    //时间间隔
    private double samplePeriod = 0.2;

    private int firstGPSOff = 0;
    private int GPSOff = 0;

    private double lastGPSLongtitude = 0, lastGPSLatitude = 0, lastGPSh = 0, lastGPSyaw = 0, lastGPSv = 0;
    private double[] Vccq = new double[3];

    //速度
    private double lastVx = 0.0, lastVy = 0.0, lastVz = 0.0;
    private double GPSVe = 0.0, GPSVn = 0.0, GPSVu = 0.0;
    //位移
    private double last_L = 0.0, last_E = 0.0, last_h = 0.0;
    private double L = 0.0, E = 0.0, h = 0.0;

    private double Rm = 0, Rn = 0, R0 = 0;
    private double tao = 1;

    private Matrix mahonyR, Fn, LineFn, GFn;
    private float[] r = new float[9];

    double Yaw = 0;

    public String dealData(Datas datas) {
        double[] result = {0, 0, 0, 0, 0, 0};

        //g
        double ax = -(datas.getAcc()[0] - accCalStc_X);
        double ay = -(datas.getAcc()[1] - accCalStc_Y);
        double az = -(datas.getAcc()[2] - accCalStc_Z);

        //g
        double lax = -datas.getLine_acc()[0];
        double lay = -datas.getLine_acc()[1];
        double laz = -datas.getLine_acc()[2];

        //度
        double gx = (datas.getGyo()[0] - gyoCalStc_X) * rad2deg;
        double gy = (datas.getGyo()[1] - gyoCalStc_Y) * rad2deg;
        double gz = (datas.getGyo()[2] - gyoCalStc_Z) * rad2deg;

        //微特斯拉
        double mx = datas.getMag()[0];
        double my = datas.getMag()[1];
        double mz = datas.getMag()[2];

        //g
        double grax = -datas.getGra()[0];
        double gray = -datas.getGra()[1];
        double graz = -datas.getGra()[2];


        NMEA_GPS_INFO ngi = datas.getNgi();
        double GPSLongitude = ngi.getLongitude() / rad2deg;
        double GPSLatitude = ngi.getLatitude() / rad2deg;
        double GPSHeight = ngi.getHeight();
        double GPSYaw = ngi.getYaw();
        double GPSv = ngi.getSpeed();

        String data2write = datas.getAcc()[0] + "," + datas.getAcc()[1] + "," + datas.getAcc()[2] + "," +
                datas.getLine_acc()[0] + "," + datas.getLine_acc()[1] + "," + datas.getLine_acc()[2] + "," +
                datas.getGra()[0] + "," + datas.getGra()[1] + "," + datas.getGra()[2] + "," +
                datas.getGyo()[0] + "," + datas.getGyo()[1] + "," + datas.getGyo()[2] + "," +
                datas.getMag()[0] + "," + datas.getMag()[1] + "," + datas.getMag()[2] + "," +
                datas.getQ()[0] + "," + datas.getQ()[1] + "," + datas.getQ()[2] + "," + datas.getQ()[3] + "," +
                datas.getPressure() + "," +
                datas.getNgi().getLongitude() + "," + datas.getNgi().getLatitude() + "," +
                datas.getNgi().getHeight() + "," + datas.getNgi().getSpeed() + "," +
                datas.getNgi().getYaw() + "," + datas.getNgi().getSN() + "," + datas.getNgi().getSN_Tosee() + "," +
                datas.getNgi().getHDOP() + "," + datas.getNgi().getUTCTime();

        if (firstGPSOff == 0) {
            if (ngi.getHDOP() > 4) {
                return data2write;
            } else if (ngi.getLatitude() != 0) {
                lastGPSLongtitude = GPSLongitude;
                lastGPSLatitude = GPSLatitude;
                lastGPSh = GPSHeight;
                lastGPSyaw = GPSYaw;
                lastGPSv = GPSv;

                last_E = GPSLongitude;
                last_L = GPSLatitude;
                last_h = GPSHeight;

                E = last_E;
                L = last_L;
                h = last_h;
                firstGPSOff = 1;

                setMahonyRbyRR(datas.getRR());

                Yaw = GPSYaw;
            }
        } else if (firstGPSOff == 1) {
            double[] data = new double[]{
                    ax, ay, az,
                    gx, gy, gz,
                    mx, my, mz,
                    lax, lay, laz,
                    grax, gray, graz
            };
            data = smooth.getSmoothResult(data);

            getRotationMatrix(r, null, new float[]{(float) data[12], (float) data[13], (float) data[14]},
                    new float[]{(float) data[6], (float) data[7], (float) data[8]});
            setMahonyRbyRR(r);

            double GG = sqrt(pow(data[12], 2) + pow(data[13], 2) + pow(data[14], 2));
            double ag = pointmulti(
                    new double[]{data[3], data[4], data[5]},
                    new double[]{data[12] / GG, data[13] / GG, data[14] / GG});
            Yaw += ag * samplePeriod;
            if (Yaw > 360)
                Yaw -= 360;
            if (Yaw < 0)
                Yaw += 360;

            Rm = earthRe * (1 - 2 * earthf + 3 * earthf * sin(last_L) * sin(last_L));
            Rn = earthRe * (1 + earthf * sin(last_L) * sin(last_L));
            R0 = sqrt(Rm * Rm + Rn * Rn);

            accToFn(data[0], data[1], data[12]);
            accToLineFn(data[9], data[10], data[11]);
            gaccTogFn(data[12], data[13], data[14]);
            Vccq[0] = LineFn.get(0, 0);
            Vccq[1] = LineFn.get(1, 0);
            Vccq[2] = LineFn.get(2, 0);

//            if (abs(Vccq[0]) > 0.1)
            lastVx += Vccq[0] * samplePeriod;
//            if (abs(Vccq[1]) > 0.1)
            lastVy += Vccq[1] * samplePeriod;
//            if (abs(Vccq[2]) > 0.1)
            lastVz += Vccq[2] * samplePeriod;

            double V_mod = sqrt(lastVx * lastVx + lastVy * lastVy);
            lastVx = V_mod * sin(Yaw * deg2rad);
            lastVy = V_mod * cos(Yaw * deg2rad);

            L += (lastVy / (Rm + last_h)) * samplePeriod;
            E += (lastVx / (cos(last_L) * (Rn + last_h))) * samplePeriod;
            h += lastVz * samplePeriod;

            //到此，INS定位计算完毕

            boolean isUsed = true;
            if (GPSv == 0 && GPSYaw == 0)
                isUsed = false;


            if (ngi.getHDOP() < 3.0 && datas.isChanged() && isUsed) {
                GPSVn = GPSv * cos(GPSYaw * PI / 180);
                GPSVe = GPSv * sin(GPSYaw * PI / 180);
                GPSVu = GPSHeight - lastGPSh;

                //这儿有当时C语言版本用来设置工作区重置数值的部分
                if (GPSOff == 1) {
                    E = GPSLongitude;
                    L = GPSLatitude;
                    h = GPSHeight;
                    GPSOff = 0;
                }
//                tao += samplePeriod;
                Matrix Dpv = new Matrix(6, 1);
                Dpv.set(0, 0, L - GPSLatitude);
                Dpv.set(1, 0, E - GPSLongitude);
                Dpv.set(2, 0, h - GPSHeight);
                Dpv.set(3, 0, lastVx - GPSVe);
                Dpv.set(4, 0, lastVy - GPSVn);
                Dpv.set(5, 0, lastVz - GPSVu);
                Matrix XX_Matrix = kf.kalman_GPS_INS_pv(Dpv, lastVx, lastVy, lastVz, last_L, last_h, mahonyR, Fn, tao, Rm, Rn);
                double[][] XX = XX_Matrix.getArray();

                lastVx -= XX[3][0];
                lastVy -= XX[4][0];
                lastVz -= XX[5][0];

                L -= XX[6][0];
                E -= XX[7][0];
                h -= XX[8][0];

                gyoCalStc_X = XX[9][0] - XX[12][0] / 300;
                gyoCalStc_Y = XX[10][0] - XX[12][0] / 300;
                gyoCalStc_Z = XX[11][0] - XX[12][0] / 300;

                accCalStc_X = -XX[15][0] / 1000;
                accCalStc_Y = -XX[16][0] / 1000;
                accCalStc_Z = -XX[17][0] / 1000;


                lastGPSLongtitude = GPSLongitude;
                lastGPSLatitude = GPSLatitude;
                lastGPSh = GPSHeight;
                lastGPSyaw = GPSYaw;
                lastGPSv = GPSv;

                double Lat_delta = L - GPSLatitude;
                double Lot_delta = E - GPSLongitude;
                double distance = 2 * asin(sqrt(pow(sin(Lat_delta / 2), 2) + cos(L) * cos(GPSLatitude) * pow(sin(Lot_delta / 2), 2))) * earthRe;

                if (distance >= 20) {
                    L = GPSLatitude;
                    E = GPSLongitude;
                    h = GPSHeight;
                    Yaw = GPSYaw;

                    lastVx = GPSVe;
                    lastVy = GPSVn;
                }
            } else if (ngi.getHDOP() >= 3.0) {
                GPSOff = 1;
            }
            last_L = L;
            last_h = h;
        }
        result = new double[]{E * rad2deg, L * rad2deg, h, lastVx, lastVy, lastVz};
        return data2write + "%%%" +
                "Lo:" + result[0] + "\nLa:" + result[1] +
                "\nHe:" + result[2] + "\nVe:" + getscale(result[3], 6) +
                "\nVn:" + getscale(result[4], 6) + "\nVu:" + getscale(result[5],6);
    }

    public double pointmulti(double[] vector1, double[] vector2) {
        return vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
    }

    public void accToFn(double x, double y, double z) {
        Matrix temp = new Matrix(3, 1);
        temp.set(0, 0, x);
        temp.set(1, 0, y);
        temp.set(2, 0, z);
        Fn = mahonyR.times(temp);
    }

    public void accToLineFn(double x, double y, double z) {
        Matrix temp = new Matrix(3, 1);
        temp.set(0, 0, x);
        temp.set(1, 0, y);
        temp.set(2, 0, z);
        LineFn = mahonyR.times(temp);
    }

    public void gaccTogFn(double x, double y, double z) {
        Matrix temp = new Matrix(3, 1);
        temp.set(0, 0, x);
        temp.set(1, 0, y);
        temp.set(2, 0, z);
        GFn = mahonyR.times(temp);
    }

    public void setMahonyRbyRR(float[] RR) {
        mahonyR = new Matrix(3, 3);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                mahonyR.set(i, i, RR[i * 3 + j]);
            }
        }
    }
    private double getscale(double num, int scale) {
        BigDecimal bd = new BigDecimal(num);
        bd = bd.setScale(scale, 4);
        return bd.doubleValue();
    }
    public static boolean getRotationMatrix(float[] R, float[] I, float[] gravity, float[] geomagnetic) {
        float Ax = gravity[0];
        float Ay = gravity[1];
        float Az = gravity[2];

        final float normsqA = (Ax * Ax + Ay * Ay + Az * Az);
        final float g = 9.81f;
        final float freeFallGravitySquared = 0.01f * g * g;
        if (normsqA < freeFallGravitySquared) {
            // gravity less than 10% of normal value
            return false;
        }

        final float Ex = geomagnetic[0];
        final float Ey = geomagnetic[1];
        final float Ez = geomagnetic[2];
        float Hx = Ey * Az - Ez * Ay;
        float Hy = Ez * Ax - Ex * Az;
        float Hz = Ex * Ay - Ey * Ax;
        final float normH = (float) Math.sqrt(Hx * Hx + Hy * Hy + Hz * Hz);

        if (normH < 0.1f) {
            // device is close to free fall (or in space?), or close to
            // magnetic north pole. Typical values are  > 100.
            return false;
        }
        final float invH = 1.0f / normH;
        Hx *= invH;
        Hy *= invH;
        Hz *= invH;
        final float invA = 1.0f / (float) Math.sqrt(Ax * Ax + Ay * Ay + Az * Az);
        Ax *= invA;
        Ay *= invA;
        Az *= invA;
        final float Mx = Ay * Hz - Az * Hy;
        final float My = Az * Hx - Ax * Hz;
        final float Mz = Ax * Hy - Ay * Hx;
        if (R != null) {
            if (R.length == 9) {
                R[0] = Hx;
                R[1] = Hy;
                R[2] = Hz;
                R[3] = Mx;
                R[4] = My;
                R[5] = Mz;
                R[6] = Ax;
                R[7] = Ay;
                R[8] = Az;
            } else if (R.length == 16) {
                R[0] = Hx;
                R[1] = Hy;
                R[2] = Hz;
                R[3] = 0;
                R[4] = Mx;
                R[5] = My;
                R[6] = Mz;
                R[7] = 0;
                R[8] = Ax;
                R[9] = Ay;
                R[10] = Az;
                R[11] = 0;
                R[12] = 0;
                R[13] = 0;
                R[14] = 0;
                R[15] = 1;
            }
        }
        if (I != null) {
            // compute the inclination matrix by projecting the geomagnetic
            // vector onto the Z (gravity) and X (horizontal component
            // of geomagnetic vector) axes.
            final float invE = 1.0f / (float) Math.sqrt(Ex * Ex + Ey * Ey + Ez * Ez);
            final float c = (Ex * Mx + Ey * My + Ez * Mz) * invE;
            final float s = (Ex * Ax + Ey * Ay + Ez * Az) * invE;
            if (I.length == 9) {
                I[0] = 1;
                I[1] = 0;
                I[2] = 0;
                I[3] = 0;
                I[4] = c;
                I[5] = s;
                I[6] = 0;
                I[7] = -s;
                I[8] = c;
            } else if (I.length == 16) {
                I[0] = 1;
                I[1] = 0;
                I[2] = 0;
                I[4] = 0;
                I[5] = c;
                I[6] = s;
                I[8] = 0;
                I[9] = -s;
                I[10] = c;
                I[3] = I[7] = I[11] = I[12] = I[13] = I[14] = 0;
                I[15] = 1;
            }
        }
        return true;
    }
}
