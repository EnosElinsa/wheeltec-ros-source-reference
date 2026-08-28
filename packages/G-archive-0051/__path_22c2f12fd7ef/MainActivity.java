package com.example.myapplication;


import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;

public class MainActivity extends Activity {
//    private RosImageView<CompressedImage> cameraView;
    private Button backButton,makeButton,navButton,rgbcameraButton;
    public static EditText set_ip_text;

    public MainActivity() {
//        super("Robot","Robot");
    }
    public static String planTopic,car_model;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        backButton = findViewById(R.id.back_button);
        makeButton = findViewById(R.id.make_map);
        navButton = findViewById(R.id.map_nav);
        rgbcameraButton = findViewById(R.id.RGB_camera);
        set_ip_text= findViewById(R.id.set_ip_text);
        backButton.setOnClickListener(view -> onBackPressed());
        RadioButton RbDwa =findViewById(R.id.DWAButton);
        RadioButton RbTeb =findViewById(R.id.TebradioButton);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,  WindowManager.LayoutParams.FLAG_FULLSCREEN); //设置全屏
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);//设置不息屏
        rgbcameraButton.setOnClickListener(v -> {
//            rgbcameraButton.setText("rgbcamera");
            startActivity(new Intent(MainActivity.this, CameraActivity.class));
        });

        makeButton.setOnClickListener(v -> {
//            makeButton.setText("make_map");
            startActivity(new Intent(MainActivity.this, MakemapActivity.class));

        });

        navButton.setOnClickListener(v -> {
            if(RbTeb.isChecked()){setTebLocalPlannerROS_topic(); }
            if(RbDwa.isChecked()){setNavfnROS_topic();}
            startActivity(new Intent(MainActivity.this, MapnavActivity.class));
                  });

    }



    public void OnClickTebLocalPlannerROS_topic(View view) {
        setTebLocalPlannerROS_topic();
    }


    public void OnClickNavfnROS_topic(View view) {
        setNavfnROS_topic();
    }
/*****
 *选择通用模式 *
 */
    public void setTebLocalPlannerROS_topic() {
        planTopic  = getString(R.string.global_TebLocalPlannerROS_topic);
        car_model=getString(R.string.global_TebLocalPlannerROS);
    }
/*****
 *选择差速模式 *
 */
    public void setNavfnROS_topic() {
        planTopic  = getString(R.string.global_NavfnROS_topic);
        car_model=getString(R.string.global_NavfnROS);
    }




//    @Override
//    protected void init(NodeMainExecutor nodeMainExecutor) {
//
//
//    }

//    @Override
//    public void onCreate(Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);
////
////        cameraView = (RosImageView<CompressedImage>) findViewById(R.id.image);
////        cameraView.setMessageType(CompressedImage._TYPE);
////        cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
////        backButton = (Button) findViewById(R.id.back_button);
////        backButton.setOnClickListener(new View.OnClickListener() {
////            @Override
////            public void onClick(View view) {
////                onBackPressed();
////            }
////        });
//
//    }

//    @Override
//    protected void init(NodeMainExecutor nodeMainExecutor) {
//        super.init(nodeMainExecutor);
//
//        try {
//            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
//            java.net.InetAddress local_network_address = socket.getLocalAddress();
//            socket.close();
//            NodeConfiguration nodeConfiguration =
//                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
//
////            java.lang.String joyTopic = remaps.get(getString(R.string.joystick_topic));
//            java.lang.String camTopic = remaps.get(getString(R.string.camera_topic));
//
//            NameResolver appNameSpace = getMasterNameSpace();
////            joyTopic = appNameSpace.resolve(joyTopic).toString();
//            camTopic = appNameSpace.resolve(camTopic).toString();
//
//            cameraView.setTopicName(camTopic);
////            virtualJoystickView.setTopicName(joyTopic);
//
//            nodeMainExecutor.execute(cameraView, nodeConfiguration
//                    .setNodeName("android/camera_view"));
//        } catch (IOException e) {
//            // Socket problem
//        }
//
//    }
//
//    @Override
//    public boolean onCreateOptionsMenu(Menu menu){
//        menu.add(0,0,0,R.string.stop_app);
//
//        return super.onCreateOptionsMenu(menu);
//    }
//
//    @Override
//    public boolean onOptionsItemSelected(MenuItem item){
//        super.onOptionsItemSelected(item);
//        switch (item.getItemId()){
//            case 0:
//                onDestroy();
//                break;
//        }
//        return true;
//    }

//    @Override
//    protected void init(NodeMainExecutor nodeMainExecutor) {
//
//    }


//    @SuppressLint("StaticFieldLeak")
//    @Override
//    public void startMasterChooser() {
//        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
//        java.lang.String masterURI = prefs.getString("masterURI", "http://192.168.0.100:11311");
//
//        Intent data = new Intent();
//        data.putExtra("ROS_MASTER_URI", masterURI);
//        onActivityResult(0, RESULT_OK, data);
//
//    }
}