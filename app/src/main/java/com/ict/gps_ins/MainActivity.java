package com.ict.gps_ins;

import android.Manifest;

import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.baidu.location.BDLocation;
import com.baidu.mapapi.SDKInitializer;
import com.baidu.mapapi.map.BaiduMap;
import com.baidu.mapapi.map.MapStatusUpdate;
import com.baidu.mapapi.map.MapStatusUpdateFactory;
import com.baidu.mapapi.map.MapView;
import com.baidu.mapapi.map.MyLocationData;
import com.baidu.mapapi.model.LatLng;
import com.ict.gps_ins.Location_Alg.Tools.GnssContainer;
import com.ict.gps_ins.Location_Alg.Tools.SensorContainer;
import com.ict.gps_ins.Location_Alg.utils.IntegratedLocation;
import com.ict.gps_ins.Location_Alg.model.Datas;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;


public class MainActivity extends AppCompatActivity {


    private Datas datas;
    private IntegratedLocation integratedLocation;


    private SensorContainer sensorContainer;

    private GnssContainer gnssContainer;

    private Timer locateTimer;

    private boolean isFirstLocated = true;


    private MapView mapView;

    private BaiduMap baiduMap;


    private Handler location_handler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            //组合导航解算
            String result = integratedLocation.dealData(datas);
            BDLocation bdLocation = new BDLocation();
            //这里引入一个经纬度解析服务？
//            bdLocation.setLatitude(1);
//            bdLocation.setLongitude(1);
//            navigateTo(bdLocation);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        SDKInitializer.initialize(getApplicationContext());

        setContentView(R.layout.activity_main);
        requestPermissions(new String[]{Manifest.permission.ACCESS_COARSE_LOCATION, Manifest.permission.ACCESS_FINE_LOCATION
                , Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE}, 1);


        mapView = findViewById(R.id.bmapView);
        baiduMap = mapView.getMap();
        baiduMap.setMyLocationEnabled(true);

        List<String> permissionList = new ArrayList<>();
        if(ContextCompat.checkSelfPermission(MainActivity.this, Manifest.permission.ACCESS_FINE_LOCATION)!= PackageManager.PERMISSION_GRANTED){
            permissionList.add(Manifest.permission.ACCESS_FINE_LOCATION);
        }
        if(ContextCompat.checkSelfPermission(MainActivity.this, Manifest.permission.READ_PHONE_STATE)!= PackageManager.PERMISSION_GRANTED){
            permissionList.add(Manifest.permission.READ_PHONE_STATE);
        }
        if(ContextCompat.checkSelfPermission(MainActivity.this, Manifest.permission.WRITE_EXTERNAL_STORAGE)!= PackageManager.PERMISSION_GRANTED){
            permissionList.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        }
        if(!permissionList.isEmpty()){
            String[] permissions = permissionList.toArray(new String[permissionList.size()]);
            requestPermissions(permissions,1);
        }

        //进行纯惯导运算
        datas = new Datas();
        //创建组合导航运算实例 todo 这里的period具体是多少？
        integratedLocation = new IntegratedLocation(0.01);

        //创建传感器容器
        sensorContainer = new SensorContainer(this, datas);
        //注册传感器
        sensorContainer.registerListeners();
        //姿态初始化
        sensorContainer.initGesture();

        //利用该实例初始化gnssContainer，在收到GNSS信号时进行组合导航运算
        gnssContainer = new GnssContainer(this,datas);
        gnssContainer.registerMeasurements();


        //定时进行组合导航解算
        this.locateTimer = new Timer();
        this.locateTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                MainActivity.this.location_handler.sendMessage(MainActivity.this.location_handler.obtainMessage());
            }
        }, 0, 10);

    }

    @Override
    protected void onResume() {
        super.onResume();
        mapView.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mapView.onPause();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        mapView.onDestroy();
        gnssContainer.unregisterMeasurements();
    }

    @Override
    protected void onStop() {
        super.onStop();
    }


    private void navigateTo(BDLocation location){
        if(isFirstLocated){
            LatLng latLng = new LatLng(location.getLatitude(),location.getLongitude());
            MapStatusUpdate mapStatusUpdate = MapStatusUpdateFactory.newLatLng(latLng);
            baiduMap.animateMapStatus(mapStatusUpdate);
            mapStatusUpdate = MapStatusUpdateFactory.zoomTo(16f);
            baiduMap.animateMapStatus(mapStatusUpdate);
            isFirstLocated = false;
        }
        MyLocationData.Builder locationBuilder = new MyLocationData.Builder();
        locationBuilder.latitude(location.getLatitude());
        locationBuilder.longitude(location.getLongitude());
        MyLocationData locationData = locationBuilder.build();
        baiduMap.setMyLocationData(locationData);

    }

}