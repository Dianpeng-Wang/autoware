package jp.tier4.autowaredrive.ui;

/**
 * Created by yuki.iida on 2017/06/28.
 */

public class CanInfo {
    private final static String TAG = "CanInfo";

    private static CanInfo instance = new CanInfo();

    String tm;
    float devmode;
    float drvcontmode;
    float drvoverridemode;
    float drvservo;
    float drivepedal;
    float targetpedalstr;
    float inputpedalstr;
    float targetveloc;
    float speed;
    float driveshift;
    float targetshift;
    float inputshift;
    float strmode;
    float strcontmode;
    float stroverridemode;
    float strservo;
    float targettorque;
    float torque;
    float angle;
    float targetangle;
    float bbrakepress;
    float brakepedal;
    float brtargetpedalstr;
    float brinputpedalstr;
    float battery;
    float voltage;
    float anp;
    float battmaxtemparature;
    float battmintemparature;
    float maxchgcurrent;
    float maxdischgcurrent;
    float sideacc;
    float accellfromp;
    float anglefromp;
    float brakepedalfromp;
    float speedfr;
    float speedfl;
    float speedrr;
    float speedrl;
    float velocfromp2;
    float drvmode;
    float devpedalstrfromp;
    float rpm;
    float velocflfromp;
    float ev_mode;
    float temp;
    float shiftfrmprius;
    float light;
    float gaslevel;
    float door;
    float cluise;

    private CanInfo(){
    }

    public static CanInfo getInstance() {
        return instance;
    }

    public void parseCanInfo(String message) {

        String[] canInfo = message.split(",", -1);

        tm = canInfo[0];
        devmode = Float.parseFloat(canInfo[1]);
        drvcontmode = Float.parseFloat(canInfo[2]);
        drvoverridemode = Float.parseFloat(canInfo[3]);
        drvservo = Float.parseFloat(canInfo[4]);
        drivepedal = Float.parseFloat(canInfo[5]);
        targetpedalstr = Float.parseFloat(canInfo[6]);
        inputpedalstr = Float.parseFloat(canInfo[7]);
        targetveloc = Float.parseFloat(canInfo[8]);
        speed = Float.parseFloat(canInfo[9]);
        driveshift = Float.parseFloat(canInfo[10]);
        targetshift = Float.parseFloat(canInfo[11]);
        inputshift = Float.parseFloat(canInfo[12]);
        strmode = Float.parseFloat(canInfo[13]);
        strcontmode = Float.parseFloat(canInfo[14]);
        stroverridemode = Float.parseFloat(canInfo[15]);
        strservo = Float.parseFloat(canInfo[16]);
        targettorque = Float.parseFloat(canInfo[17]);
        torque = Float.parseFloat(canInfo[18]);
        angle = Float.parseFloat(canInfo[19]);
        targetangle = Float.parseFloat(canInfo[20]);
        bbrakepress = Float.parseFloat(canInfo[21]);
        brakepedal = Float.parseFloat(canInfo[22]);
        brtargetpedalstr = Float.parseFloat(canInfo[23]);
        brinputpedalstr = Float.parseFloat(canInfo[24]);
        battery = Float.parseFloat(canInfo[25]);
        voltage = Float.parseFloat(canInfo[26]);
        anp = Float.parseFloat(canInfo[27]);
        battmaxtemparature = Float.parseFloat(canInfo[28]);
        battmintemparature = Float.parseFloat(canInfo[29]);
        maxchgcurrent = Float.parseFloat(canInfo[30]);
        maxdischgcurrent = Float.parseFloat(canInfo[31]);
        sideacc = Float.parseFloat(canInfo[32]);
        accellfromp = Float.parseFloat(canInfo[33]);
        anglefromp = Float.parseFloat(canInfo[34]);
        brakepedalfromp = Float.parseFloat(canInfo[35]);
        speedfr = Float.parseFloat(canInfo[36]);
        speedfl = Float.parseFloat(canInfo[37]);
        speedrr = Float.parseFloat(canInfo[38]);
        speedrl = Float.parseFloat(canInfo[39]);
        velocfromp2 = Float.parseFloat(canInfo[40]);
        drvmode = Float.parseFloat(canInfo[41]);
        devpedalstrfromp = Float.parseFloat(canInfo[42]);
        rpm = Float.parseFloat(canInfo[43]);
        velocflfromp = Float.parseFloat(canInfo[44]);
        ev_mode = Float.parseFloat(canInfo[45]);
        temp = Float.parseFloat(canInfo[46]);
        shiftfrmprius = Float.parseFloat(canInfo[47]);
        light = Float.parseFloat(canInfo[48]);
        gaslevel = Float.parseFloat(canInfo[49]);
        door = Float.parseFloat(canInfo[50]);
        cluise = Float.parseFloat(canInfo[51]);
    }
}
