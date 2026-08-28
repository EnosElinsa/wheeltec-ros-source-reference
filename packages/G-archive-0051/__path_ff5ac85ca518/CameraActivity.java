package com.example.myapplication;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

import com.bumptech.glide.Glide;
import com.bumptech.glide.load.engine.DiskCacheStrategy;
import com.bumptech.glide.request.RequestOptions;
import com.github.rosjava.android_remocons.common_tools.apps.RosAppActivity;

import org.ros.android.BitmapFromCompressedImage;
import org.ros.android.view.RosImageView;
import org.ros.android.view.VirtualJoystickView;
import org.ros.namespace.NameResolver;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

import sensor_msgs.CompressedImage;

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class CameraActivity extends RosAppActivity {
    private RosImageView<CompressedImage> cameraView;
    private VirtualJoystickView virtualJoystickView;
    private Button backButton;
    private TextView textView;
//    private ImageView topimage;

    public CameraActivity() {
        // The RosActivity constructor configures the notification title and ticker messages.
        super("android teleop", "android teleop");
    }

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState) {

        setDashboardResource(R.id.top_bar);
        setMainWindowResource(R.layout.activity_camera);
        super.onCreate(savedInstanceState);

        cameraView = (RosImageView<sensor_msgs.CompressedImage>) findViewById(R.id.image);
        cameraView.setMessageType(sensor_msgs.CompressedImage._TYPE);
        cameraView.setMessageToBitmapCallable(new BitmapFromCompressedImage());
        virtualJoystickView = (VirtualJoystickView) findViewById(R.id.virtual_joystick);
        backButton = (Button) findViewById(R.id.back_button);
        textView=findViewById(R.id.IP);
//        topimage=findViewById(R.id.top_image);
        textView.setText("小车的IP："+MainActivity.set_ip_text.getText());
        RequestOptions options = new RequestOptions()
                .diskCacheStrategy(DiskCacheStrategy.RESOURCE);
        Glide.with(CameraActivity.this).load(R.drawable.camera).apply(options).into(cameraView);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);//设置不息屏
        backButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
//                onDestroy();
//                srChooser();
                onBackPressed();
            }
        });
    }

    /*
     * 重写startMasterChooser
     *登录
     * */
    @SuppressLint("StaticFieldLeak")
    @Override
    public void startMasterChooser() {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
        java.lang.String masterURI = prefs.getString("masterURI", "http://"+MainActivity.set_ip_text.getText()+":11311");

        Intent data = new Intent();
        data.putExtra("ROS_MASTER_URI", masterURI);
        onActivityResult(0, RESULT_OK, data);
    }

    public void srChooser() {
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getBaseContext());
        java.lang.String masterURI = prefs.getString("masterURI", "http://"+"546.125.2.23"+":11311");

        Intent data = new Intent();
        data.putExtra("ROS_MASTER_URI", masterURI);
        onActivityResult(0, RESULT_OK, data);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {

        super.init(nodeMainExecutor);

        try {
            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();
            socket.close();
            NodeConfiguration nodeConfiguration =
                    NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());

            String joyTopic = remaps.get(getString(R.string.joystick_topic));
            String camTopic = remaps.get(getString(R.string.camera_topic));

            NameResolver appNameSpace = getMasterNameSpace();
            joyTopic = appNameSpace.resolve(joyTopic).toString();
            camTopic = appNameSpace.resolve(camTopic).toString();

            cameraView.setTopicName(camTopic);
            virtualJoystickView.setTopicName(joyTopic);

            nodeMainExecutor.execute(cameraView, nodeConfiguration
                    .setNodeName("android/camera_view"));
            nodeMainExecutor.execute(virtualJoystickView,
                    nodeConfiguration.setNodeName("android/virtual_joystick"));
//            topimage.setVisibility(topimage.INVISIBLE);
//            ViewGroup.LayoutParams layoutParams = cameraView.getLayoutParams();
//            layoutParams.height = ViewGroup.LayoutParams.MATCH_PARENT;
//            layoutParams.width = ViewGroup.LayoutParams.MATCH_PARENT;
        } catch (IOException e) {
            // Socket problem
        }

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu){
        menu.add(0,0,0,R.string.stop_app);

        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item){
        super.onOptionsItemSelected(item);
        switch (item.getItemId()){
            case 0:
                onDestroy();
                break;
        }
        return true;
    }
}
