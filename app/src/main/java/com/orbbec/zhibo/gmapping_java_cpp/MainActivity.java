package com.orbbec.zhibo.gmapping_java_cpp;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    // Used to load the 'native-lib' library on application startup.
    static {
        System.loadLibrary("slam-lib");
//        System.loadLibrary("gridfastslam");
//        System.loadLibrary("scanmatcher");
//        System.loadLibrary("utils");
//        System.loadLibrary("sensor_base");
//        System.loadLibrary("sensor_odometry");
//        System.loadLibrary("sensor_range");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Example of a call to a native method
        TextView tv = (TextView) findViewById(R.id.sample_text);
        tv.setText(stringFromJNI());
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();
}
