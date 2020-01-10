package com.ict.gps_ins.Location_Alg.Tools;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.Message;
import android.widget.Toast;

import com.ict.gps_ins.Location_Alg.model.Datas;
import com.ict.gps_ins.Location_Alg.utils.IntegratedLocation;
import com.ict.gps_ins.MainActivity;

import java.util.Timer;
import java.util.TimerTask;

public class SensorContainer {

    private final SensorManager sensorManager;

    private final Datas datas;

    private float[] r = new float[9];

    private Timer initTimer;

    private int init_time = 0;

    SensorEventListener sensorEventListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent event) {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_ACCELEROMETER:
//                    acc_string = "加速度计:\nx=" + Tools.getScale(event.values[0], 6) + "\ny=" + Tools.getScale(event.values[1], 6) + "\nz=" + Tools.getScale(event.values[2], 6);
                    datas.setAcc(new double[]{event.values[0], event.values[1], event.values[2]});
//                    if (currIndex == 1) {
//                        addEntry(acc_index, event.values[0], 0, 0);
//                        addEntry(acc_index, event.values[1], 0, 1);
//                        addEntry(acc_index, event.values[2], 0, 2);
//                        acc_index++;
//                    } else {
//                        acc_index = 0;
//                        clearChart(0);
//                    }
                    break;
                case Sensor.TYPE_GYROSCOPE:
//                    gyo_string = "陀螺仪:\nx=" + getscale(event.values[0], 7) + "\ny=" + getscale(event.values[1], 7) + "\nz=" + getscale(event.values[2], 7);
                    datas.setGyo(new double[]{event.values[0], event.values[1], event.values[2]});
//                    if (currIndex == 3) {
//                        addEntry(gyo_index, event.values[0], 2, 0);
//                        addEntry(gyo_index, event.values[1], 2, 1);
//                        addEntry(gyo_index, event.values[2], 2, 2);
//                        gyo_index++;
//                    } else {
//                        gyo_index = 0;
//                        clearChart(2);
//                    }
                    break;
                case Sensor.TYPE_MAGNETIC_FIELD:
//                    mag_string = "磁力计:\nx=" + getscale(event.values[0], 7) + "\ny=" + getscale(event.values[1], 7) + "\nz=" + getscale(event.values[2], 7);
                    datas.setMag(new double[]{event.values[0], event.values[1], event.values[2]});
//                    if (currIndex == 4) {
//                        addEntry(mag_index, event.values[0], 3, 0);
//                        addEntry(mag_index, event.values[1], 3, 1);
//                        addEntry(mag_index, event.values[2], 3, 2);
//                        mag_index++;
//                    } else {
//                        mag_index = 0;
//                        clearChart(3);
//                    }
                    break;
                case Sensor.TYPE_LINEAR_ACCELERATION:
//                    lineacc_string = "线性加速度计:\nx=" + getscale(event.values[0], 6) + "\ny=" + getscale(event.values[1], 6) + "\nz=" + getscale(event.values[2], 6);
                    datas.setLine_acc(new double[]{event.values[0], event.values[1], event.values[2]});
//                    if (currIndex == 2) {
//                        addEntry(lineacc_index, event.values[0], 1, 0);
//                        addEntry(lineacc_index, event.values[1], 1, 1);
//                        addEntry(lineacc_index, event.values[2], 1, 2);
//                        lineacc_index++;
//                    } else {
//                        lineacc_index = 0;
//                        clearChart(1);
//                    }
                    break;
                case Sensor.TYPE_GRAVITY:
                    datas.setGra(new double[]{event.values[0], event.values[1], event.values[2]});
                    break;
                case Sensor.TYPE_ROTATION_VECTOR:
                    datas.setQ(new double[]{event.values[3], event.values[0], event.values[1], event.values[2]});
                    break;
                case Sensor.TYPE_PRESSURE:
                    datas.setPressure(event.values[0]);
                    break;
            }

        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {

        }
    };

    private Handler init_handler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            sensorManager.getRotationMatrix(r, null, new float[]{(float) datas.getGra()[0], (float) datas.getGra()[1], (float) datas.getGra()[2]},
                    new float[]{(float) datas.getMag()[0], (float) datas.getMag()[1], (float) datas.getMag()[2]});
            float[] values = new float[3];
            sensorManager.getOrientation(r, values);
            datas.setAngle(values);
            datas.setRR(r);

            init_time++;
            if (init_time >= 1000) {
                initTimer.cancel();
            }
        }
    };

    public SensorContainer(Context context, Datas datas){
        this.sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        this.datas = datas;
    }

    public void registerListeners(){
        sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(sensorEventListener, sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR), SensorManager.SENSOR_DELAY_GAME);
    }

    public void initGesture(){
        //确定初始姿态
        this.initTimer = new Timer();
        this.initTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                init_handler.sendMessage(init_handler.obtainMessage());
            }
        }, 0, 10);
    }



}
