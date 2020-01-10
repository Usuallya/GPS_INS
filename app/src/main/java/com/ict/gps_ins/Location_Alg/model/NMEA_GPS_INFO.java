package com.ict.gps_ins.Location_Alg.model;

public class NMEA_GPS_INFO {
    private double Longitude;
    private double Latitude;
    private double Height;
    private double SN;
    private double SN_Tosee;
    private double Speed;
    private double Yaw;
    private double HDOP;
    private double UTCTime;

    public NMEA_GPS_INFO() {
    }

    public NMEA_GPS_INFO(double longitude, double latitude, double height,
                         double SN, double SN_Tosee, double speed,
                         double yaw, double HDOP, double UTCTime) {
        Longitude = longitude;
        Latitude = latitude;
        Height = height;
        this.SN = SN;
        this.SN_Tosee = SN_Tosee;
        Speed = speed;
        Yaw = yaw;
        this.HDOP = HDOP;
        this.UTCTime = UTCTime;
    }

    public double getLongitude() {
        return Longitude;
    }

    public void setLongitude(double longitude) {
        Longitude = longitude;
    }

    public double getLatitude() {
        return Latitude;
    }

    public void setLatitude(double latitude) {
        Latitude = latitude;
    }

    public double getHeight() {
        return Height;
    }

    public void setHeight(double height) {
        Height = height;
    }

    public double getSN() {
        return SN;
    }

    public void setSN(double SN) {
        this.SN = SN;
    }

    public double getSN_Tosee() {
        return SN_Tosee;
    }

    public void setSN_Tosee(double SN_Tosee) {
        this.SN_Tosee = SN_Tosee;
    }

    public double getSpeed() {
        return Speed;
    }

    public void setSpeed(double speed) {
        Speed = speed;
    }

    public double getYaw() {
        return Yaw;
    }

    public void setYaw(double yaw) {
        Yaw = yaw;
    }

    public double getHDOP() {
        return HDOP;
    }

    public void setHDOP(double HDOP) {
        this.HDOP = HDOP;
    }

    public double getUTCTime() {
        return UTCTime;
    }

    public void setUTCTime(double UTCTime) {
        this.UTCTime = UTCTime;
    }
}
