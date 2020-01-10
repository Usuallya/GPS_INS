package com.ict.gps_ins.Location_Alg.model;

public class Datas {
    private double[] acc;
    private double[] gyo;
    private double[] mag;
    private double[] line_acc;
    private double[] gra;
    private double[] Q;
    private float[] RR;
    private float[] Angle;
    private double Pressure;
    private NMEA_GPS_INFO ngi;

    private boolean isChanged = false;

    public Datas() {
        acc = new double[3];
        gyo = new double[3];
        mag = new double[3];
        line_acc = new double[3];
        gra = new double[3];
        Q = new double[4];
        RR = new float[9];
        Angle = new float[3];
        Pressure = 0;
        ngi = new NMEA_GPS_INFO();
    }

    public Datas(double[] acc, double[] gyo, double[] mag, double[] line_acc, double[] gra,
                 double[] Q, float[] RR, float[] Angle, double Pressure, NMEA_GPS_INFO ngi) {
        this.acc = acc;
        this.gyo = gyo;
        this.mag = mag;
        this.ngi = ngi;
        this.line_acc = line_acc;
        this.gra = gra;
        this.Q = Q;
        this.RR = RR;
        this.Angle = Angle;
        this.Pressure = Pressure;
    }

    public double[] getAcc() {
        return acc;
    }

    public void setAcc(double[] acc) {
        this.acc = acc;
    }

    public double[] getGyo() {
        return gyo;
    }

    public void setGyo(double[] gyo) {
        this.gyo = gyo;
    }

    public double[] getMag() {
        return mag;
    }

    public void setMag(double[] mag) {
        this.mag = mag;
    }

    public double[] getLine_acc() {
        return line_acc;
    }

    public void setLine_acc(double[] line_acc) {
        this.line_acc = line_acc;
    }

    public double[] getGra() {
        return gra;
    }

    public void setGra(double[] gra) {
        this.gra = gra;
    }

    public double[] getQ() {
        return Q;
    }

    public void setQ(double[] q) {
        Q = q;
    }

    public float[] getRR() {
        return RR;
    }

    public void setRR(float[] RR) {
        this.RR = RR;
    }

    public float[] getAngle() {
        return Angle;
    }

    public void setAngle(float[] angle) {
        Angle = angle;
    }

    public double getPressure() {
        return Pressure;
    }

    public void setPressure(double pressure) {
        Pressure = pressure;
    }

    public boolean isChanged() {
        return isChanged;
    }

    public void setChanged(boolean changed) {
        isChanged = changed;
    }

    public NMEA_GPS_INFO getNgi() {
        return ngi;
    }

    public void setNgi(NMEA_GPS_INFO ngi) {
        this.ngi = ngi;
    }
}
