package com.ict.gps_ins.Location_Alg.utils;

import java.math.BigDecimal;

import static java.lang.Math.PI;
import static java.lang.Math.asin;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class Tools {

    public static double getDistance(double Lo1, double La1, double Lo2, double La2) {
        Lo1 = Lo1 * PI / 180;
        La1 = La1 * PI / 180;
        Lo2 = Lo2 * PI / 180;
        La2 = La2 * PI / 180;
        double Lat_delta = La1 - La2;
        double Lot_delta = Lo1 - Lo2;
        return 2 * asin(sqrt(pow(sin(Lat_delta / 2), 2) + cos(La1) * cos(La2) * pow(sin(Lot_delta / 2), 2))) * 6378137;
    }

    public static float getScale(float num, int scale) {
        BigDecimal bd = new BigDecimal((double) num);
        bd = bd.setScale(scale, 4);
        return bd.floatValue();
    }
}
