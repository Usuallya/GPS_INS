package com.ict.gps_ins.Location_Alg.Tools;

import android.content.Context;
import android.location.GnssMeasurementsEvent;
import android.location.GnssNavigationMessage;

import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.OnNmeaMessageListener;
import android.os.Bundle;


import com.ict.gps_ins.Location_Alg.model.Datas;

import java.util.concurrent.TimeUnit;

/**
 * 各类位置信息的监听器容器
 */
public class GnssContainer {

    private static final long LOCATION_RATE_GPS_MS = TimeUnit.SECONDS.toMillis(1L);
    private static final long LOCATION_RATE_NETWORK_MS = TimeUnit.SECONDS.toMillis(60L);


    private final LocationManager mLocationManager;

    private final Datas datas;

    private final LocationListener mLocationListener =
            new LocationListener() {

                @Override
                public void onProviderEnabled(String provider) {

                }

                @Override
                public void onProviderDisabled(String provider) {

                }

                @Override
                public void onLocationChanged(Location location) {
                    //记录位置信息
                }

                @Override
                public void onStatusChanged(String provider, int status, Bundle extras) {

                }
            };

    private final GnssMeasurementsEvent.Callback gnssMeasurementsEventListener =
            new GnssMeasurementsEvent.Callback() {
                @Override
                public void onGnssMeasurementsReceived(GnssMeasurementsEvent event) {
                    //接收到Gnss定位信息，计算其伪距
                    event.getMeasurements();

                }

                @Override
                public void onStatusChanged(int status) {

                }
            };

    private final GnssNavigationMessage.Callback gnssNavigationMessageListener =
            new GnssNavigationMessage.Callback() {
                @Override
                public void onGnssNavigationMessageReceived(GnssNavigationMessage event) {
                    //todo 接收到导航电文，需要据此计算电离层参数，钟差等信息并保存
                }

                @Override
                public void onStatusChanged(int status) {

                }
            };


    private final OnNmeaMessageListener nmeaListener =
            new OnNmeaMessageListener() {
                @Override
                public void onNmeaMessage(String s, long l) {
                    //记录NMEA信息
                }
            };

    public GnssContainer(Context context, Datas datas) {
        this.mLocationManager = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
        this.datas = datas;
    }

    public void registerLocation() {
        boolean isGpsProviderEnabled = mLocationManager.isProviderEnabled(LocationManager.GPS_PROVIDER);
        if (isGpsProviderEnabled) {
            mLocationManager.requestLocationUpdates(
                    LocationManager.NETWORK_PROVIDER,
                    LOCATION_RATE_NETWORK_MS,
                    0.0f /* minDistance */,
                    mLocationListener);
            mLocationManager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER,
                    LOCATION_RATE_GPS_MS,
                    0.0f /* minDistance */,
                    mLocationListener);
        }
    }


    public void unregisterLocation() {
        mLocationManager.removeUpdates(mLocationListener);
    }

    public void registerMeasurements() {
        mLocationManager.registerGnssMeasurementsCallback(gnssMeasurementsEventListener);
    }

    public void unregisterMeasurements() {
        mLocationManager.unregisterGnssMeasurementsCallback(gnssMeasurementsEventListener);
    }

    public void registerNavigation() {
        mLocationManager.registerGnssNavigationMessageCallback(gnssNavigationMessageListener);
    }

    public void unregisterNavigation() {
        mLocationManager.unregisterGnssNavigationMessageCallback(gnssNavigationMessageListener);
    }

    public void registerGnssStatus() {
//        logRegistration("GnssStatus", mLocationManager.registerGnssStatusCallback(gnssStatusListener));
    }


    public void registerNmea() {
        mLocationManager.addNmeaListener(nmeaListener);
    }

    public void unregisterNmea() {
        mLocationManager.removeNmeaListener(nmeaListener);
    }

    public void registerAll() {
        registerLocation();
        registerMeasurements();
        registerNavigation();
        registerGnssStatus();
        registerNmea();
    }

    public void unregisterAll() {
        unregisterLocation();
        unregisterMeasurements();
        unregisterNavigation();
        unregisterNmea();
    }


}

