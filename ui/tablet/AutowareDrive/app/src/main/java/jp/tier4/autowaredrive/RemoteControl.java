package jp.tier4.autowaredrive;

import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.graphics.Point;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ImageButton;
import android.widget.ProgressBar;
import android.widget.SeekBar;
import android.widget.ToggleButton;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

/**
 * Created by yuki.iida on 2017/06/20.
 */

public class RemoteControl extends AppCompatActivity implements View.OnTouchListener , CompoundButton.OnCheckedChangeListener {
    private final static String TAG = "RemoteControl";

    static private String CLIENTID_HEAD = "remote_";
    static private String TOPIC = "vehicle/";
    static private int REMOTE_MODE = 4;
    static private int AUTO_MODE = 3;
    static private int EMERGENCY_MODE = 1;
    static private int NONEMERGENCY_MODE = 0;
    static private int STEERING_MAX_VAL = 720;

    private int mVehicleId;
    /*** MQTT ***/
    private MqttAndroidClient mqttAndroidClient;
    private MqttConnectOptions mqttConnectOptions;
    private String mqttId;
    private String mqttHost;
    private int mqttPort;
    private String mqttBrokerURI;
    private String mqttTopic;

    /*** UI ***/
    private ProgressBar accelBar, brakeBar;
    private Button accelBrakeButton;
    private ToggleButton remoteControlButton, emergencyButton;
    private int displayHeight, displayWidth;
    private SeekBar steeringBar;

    /*** Steering ***/
    private ImageButton steeringImageButton;
    private Bitmap steeringBitmap;
    private int steeringBitmapWidth;
    private int steeringBitmapHeight;
    private float currentSteeringAngle = 0;

    /*** Control Comand ***/
    private ControlComand mControlComand;

    /*** Thread ***/
    ControlComandUploader mControlComandUploader = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_remote_control);

        Intent intent = getIntent();
        Bundle bundle = intent.getExtras();
        if (bundle != null) {
            this.mqttHost = bundle.getString("address");
            this.mqttPort = bundle.getInt("port");
            this.mqttBrokerURI = "tcp://" + mqttHost + ":" + mqttPort;
            this.mVehicleId = bundle.getInt("vehicle_id");
            this.mqttTopic = TOPIC + mVehicleId + "/remote_cmd";
            this.mqttId = CLIENTID_HEAD + mVehicleId;

            Log.i("RemoteControl", this.mqttBrokerURI + ", " + this.mqttTopic + ", " + this.mqttId);
        }

        mqttAndroidClient = new MqttAndroidClient(this, mqttBrokerURI, mqttId);
        mqttConnectOptions = new MqttConnectOptions();

        mqttAndroidClient.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                Log.i(TAG, "Connection was lost!");
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                Log.i(TAG, "Message Arrived!: " + topic + ": " + new String(message.getPayload()));
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
                Log.i(TAG, "Delivery Complete!");
            }
        });

        /*** MQTT Brokerへ接続 ***/
        connectMqttBroker();

        mControlComand = ControlComand.getInstance();

        /*** UI ***/
        accelBrakeButton = (Button)this.findViewById(R.id.accel_brake_button);
        accelBrakeButton.setOnTouchListener(this);

        steeringImageButton = (ImageButton) findViewById(R.id.steering_image_button);

        accelBar = (ProgressBar) findViewById(R.id.accel_progressbar);
        accelBar.setMax(100);
        accelBar.setProgress(0);

        brakeBar = (ProgressBar) findViewById(R.id.brake_progressbar);
        brakeBar.setMax(100);
        brakeBar.setProgress(0);

        steeringBar = (SeekBar) findViewById(R.id.steering_seekbar);
        steeringBar.setMax(100);
        steeringBar.setProgress(50);

        WindowManager wm = (WindowManager)getSystemService(WINDOW_SERVICE);
        Display disp = wm.getDefaultDisplay();
        Point size = new Point();
        disp.getSize(size);
        displayWidth = size.x;
        Log.i(TAG, "Display = " + size.x);
        displayHeight = size.y - getStatusBarHeight();

        remoteControlButton = (ToggleButton)findViewById(R.id.remote_control_button);
        remoteControlButton.setOnCheckedChangeListener(this);

        emergencyButton = (ToggleButton)findViewById(R.id.emergency_button);
        emergencyButton.setOnCheckedChangeListener(this);

        mControlComandUploader = new ControlComandUploader(mqttAndroidClient, mqttConnectOptions, mqttTopic);
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        int buttonWidth = findViewById(R.id.steering_image_button).getWidth();
        int buttonHeight = findViewById(R.id.steering_image_button).getHeight();

        setSteeringImageButtonSize(buttonWidth, buttonHeight);
    }

    public void setSteeringImageButtonSize(int buttonWidth, int buttonHeight) {
//        steeringBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.steering);
        Bitmap steeringBitmapRaw = BitmapFactory.decodeResource(getResources(), R.drawable.steering);
        float ratio = 0;
        Matrix matrix = new Matrix();
        if(buttonWidth > buttonHeight) {
            ratio = (float) buttonHeight / (float)steeringBitmapRaw.getHeight();
        }
        else {
            ratio = (float) buttonWidth / (float)steeringBitmapRaw.getWidth();
        }
        matrix.preScale(ratio, ratio);

        steeringBitmap = Bitmap.createBitmap(steeringBitmapRaw, 0, 0, steeringBitmapRaw.getWidth(), steeringBitmapRaw.getHeight(), matrix, true);
        steeringImageButton.setImageBitmap(steeringBitmap);
        steeringBitmapWidth = steeringBitmap.getWidth();
        steeringBitmapHeight = steeringBitmap.getHeight();
        steeringImageButton.setOnTouchListener(this);
    }

    public void setSteeringAngle(float angle) {
        Matrix matrix = new Matrix();
        matrix.setRotate(angle, steeringBitmapWidth/2, steeringBitmapHeight/2);
        Bitmap bitmap2 = Bitmap.createBitmap(steeringBitmap, 0, 0, steeringBitmapWidth, steeringBitmapHeight, matrix, true);
        steeringImageButton.setImageBitmap(bitmap2);
        currentSteeringAngle = angle;
    }

    public float getSteeringAngle(float x, float y) {
        float target_angle = 0;
        
        target_angle = currentSteeringAngle + 45;
        return target_angle;
    }

    public int getStatusBarHeight() {
        int result = 0;
        int resourceId = getResources().getIdentifier("status_bar_height", "dimen", "android");
        if (resourceId > 0) {
            result = getResources().getDimensionPixelSize(resourceId);
        }
        return result;
    }

    /*** MQTT Brokerと接続 ***/
    private void connectMqttBroker() {
        try {
            mqttAndroidClient.connect(mqttConnectOptions, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Log.i(TAG, "Connection Success! isConnected: " + mqttAndroidClient.isConnected());
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Log.i(TAG, "Connection Failure! isConnected: " + exception.getMessage() + ", " + mqttAndroidClient.isConnected());
                }
            });
        } catch (MqttException ex) {
            Log.e(TAG, ex.toString());
        }
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        // Acc/Brake
        if(v == accelBrakeButton) {
            if(event.getAction() == MotionEvent.ACTION_DOWN ||
                    event.getAction() == MotionEvent.ACTION_MOVE) {
                float rate = 100 - event.getY() / displayHeight * 100;
                if(rate >= 50) {
                    mControlComand.accelCmd = (rate - 50) / 50;
                    if(mControlComand.accelCmd > 1)
                        mControlComand.accelCmd = (float)1.0;
                    mControlComand.brakeCmd = 0;
                }
                else {
                    mControlComand.accelCmd = 0;
                    mControlComand.brakeCmd = (50 - rate) / 50;
                    if(mControlComand.accelCmd > 1)
                        mControlComand.accelCmd = (float)1.0;
                }
            }
            else {
//                mControlComand.accelCmd = 0;
                mControlComand.brakeCmd = 0;
            }

            accelBar.setProgress(Math.round(mControlComand.accelCmd * 100));
            brakeBar.setProgress(Math.round(mControlComand.brakeCmd * 100));

        }
        else if(v == steeringImageButton) {
            float touch_angle = getSteeringAngle(event.getX(), event.getY());
            Log.i(TAG, "Touch " + touch_angle);
            setSteeringAngle(touch_angle);
        }
//        // Steering
//        else if(v == steeringButton) {
//            float buttonWidth = (float) (displayWidth * (8.0 / 9.0) * (5.0 / 7.0));
//            mControlComand.steeringCmd = (event.getX() - buttonWidth / 2) / (buttonWidth / 2);
//            Log.i(TAG, "Accel: " + mControlComand.accelCmd + ", Brake: " + mControlComand.brakeCmd + ", Steering: " + mControlComand.steeringCmd);
//
//            if(mControlComand.steeringCmd > 1)
//                mControlComand.steeringCmd = (float) 1.0;
//            else if(mControlComand.steeringCmd < -1.0)
//                mControlComand.steeringCmd = (float) -1.0;
//            steeringBar.setProgress(Math.round(50 + mControlComand.steeringCmd / 2 * 100));
//        }

        return false;
    }

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        if(buttonView == remoteControlButton) {
            if(mqttAndroidClient.isConnected() == false) {
                connectMqttBroker();
            }
           if(isChecked == true) {
               mControlComand.modeCmd = REMOTE_MODE;
               if(mControlComandUploader == null) {
                   mControlComandUploader = new ControlComandUploader(mqttAndroidClient, mqttConnectOptions, mqttTopic);
               }
               mControlComandUploader.execute();
           }
           else {
               mControlComand.modeCmd = AUTO_MODE;
               mControlComandUploader.setAutoMode();
               mControlComandUploader.stopLogUpload();
               mControlComandUploader = null;
           }
        }
        else if(buttonView == emergencyButton) {
            if(isChecked == true) {
                mControlComand.emergencyCmd = EMERGENCY_MODE;
            }
            else {
                mControlComand.emergencyCmd = NONEMERGENCY_MODE;
            }
        }
        Log.i(TAG, "Toggle: modeCmd = " +  mControlComand.modeCmd + ", emegencyCmd = " + mControlComand.emergencyCmd);
    }

    @Override
    public void onDestroy(){
        super.onDestroy();

//        /*** MQTTの接続を解除 ***/
//        try{
//            if(mqttAndroidClient.isConnected()) {
//                mqttAndroidClient.disconnect();
//                Log.i(TAG, "MQTT disconnect");
//            }
////            mqttAndroidClient.unregisterResources();
//
//        } catch(MqttException e) {
//            Log.e(TAG, e.toString());
//        }
    }
}
