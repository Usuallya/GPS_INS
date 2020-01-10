package com.ict.gps_ins.Location_Alg.Tools;

import java.util.ArrayList;

public class Smooth {
    private int index = 0;
    private int length = 0;
    private ArrayList<double[]> smoothRes = new ArrayList<>();

    //每次添加一个新的数据，进行数据平滑处理
    public double[] getSmoothResult(double[] data) {
        double[] temp = new double[data.length];
        if (length < 12) {
            smoothRes.add(data);
            length++;
        } else {
            smoothRes.set(index, data);
        }
        index = (index + 1) % 12;
        for (int i = 0; i < data.length; i++) {
            for (int j = 0; j < length; j++) {
                temp[i] += smoothRes.get(j)[i];
            }
            temp[i] /= length;
        }
        return temp;
    }
}
