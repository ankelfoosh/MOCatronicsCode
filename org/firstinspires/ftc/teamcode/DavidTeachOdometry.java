// This is bassically a bunch of notes and stuff that I have learned from articles and videos
// First import all the stuff we need 
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "OdometrySample")

public class OdometrySample {

    // constants
    public static final Double TRIGGER_THRESHOLD = 0.5;

    // drive base motors
    private DcMotor Lf;
    private DcMotor Rf;
    private DcMotor Lb;
    private DcMotor Rb;
    private IMU imu;

    // odometers
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderAux


    private HardwareMap hardwareMap;

    public OdometrySample(HardwareMap aHardwareMap) {

        hardwareMap = aHardwareMap;

        // configure the drive motors
        motorLf = hardwareMap.dcMotor.get("motorLF");
        motorLf.setDirection(DcMotor.Direction.FORWARD);
        motorLf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLb = hardwareMap.dcMotor.get("motorLb");
        motorLb.setDirection(DcMotor.Direction.FORWARD);
        motorLb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRf = hardwareMap.dcMotor.get("motorRF");
        motorRf.setDirection(DcMotor.Direction.REVERSE);
        motorRf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRl = hardwareMap.dcMotor.get("motorRl");
        motorRl.setDirection(DcMotor.Direction.REVERSE);
        motorRl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // link the motors with the odo encoders
        encoderLeft = motorLb;
        encoderRight = motorRb;
        encoderAux = motorRF;

        stop();
        resetDriveEncoders();

    }

    public void resetDriveEncoders() {
        motorLf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void stop() {
        motorLf.setPower(0);
        motorLb.setPower(0);
        motorRf.setPower(0);
        motorRb.setPower(0);
    }

    // constants that define the geometry of the robot and stuff
    final static double L = 20.12;       // distance between encoder 1 and encoder 2 in cm
    final static double B = 11.5;       // distance between the midpoint of encoder 1 and 2 and encoder 3
    final static double R = 3.3;        // odometry wheel radius in cm
    final static double N = 8192;       // encoder ticks per revolution, REV encoder
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    // keep track of the odometry encoders from updates
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentAuxPosition = 0;

    private int oldRightPosition = 0;
    private int oldLeftPosition = 0;
    private int oldAuxPosition = 0;


    /********************************************************************
     * Odometry Notes:
     * n1, n2, n3 are encoder values for the left. right and back (aux) pod wheel thingies
     * dn1, dn2, dn3 are the differences of encoder values between the two reads (d is delta)
     * dx, dy, dtheata describe the robot movement between the two reads (in robot coordinates)
     * X, Y, Theta are the coordinates on the field and the heading of the robot
     * also a tuple is a thing from python but this is java so im confused but anyway moving on
     ********************************************************************/

    // XyhVector is (x,y,h) where h is the heading of the robot

    public XyhVector START_POS = new XyhVector(x:213, y:102, Math.toRadians(-174));
    public XyhVector pos = new XyhVector(START_POS);

    public void odometry() {
        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldAuxPosition = currentAuxPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentAuxPosition = encoderAux.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentAuxPosition - oldAuxPosition;

        // the robot has moved a turned a bit between the measurements
        double dtheta = cm_per_tick * (dn2-dn1) / L;
        double dx = cm_per_tick * (dn1+dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2-dn1) * B / L );

        // small movement of the robot gets added to the field cordinate system thing we set up earlier
        double theta = pos.h + (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;

        //
    }

}
