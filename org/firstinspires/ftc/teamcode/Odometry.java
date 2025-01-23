package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


@Autonomous(name = "Odometry")
public class Odometry extends LinearOpMode {

  private DcMotor Lb;
  private DcMotor Lf;
  private DcMotor Rb;
  private DcMotor Rf;
  private DcMotor pickmeup;
  private DcMotor Llin;
  private DcMotor Rlin;
  private DistanceSensor Sensor;
  private IMU imu;

  private Servo imaTouchu;
  private Servo ankel;

  double tempSpeed;
  double max;
  double min;
  float yeeyaw;

  /**
   * Describe this function...
   */
  private void initialize() {
    // Recalibrates the Odometry Computer's internal IMU. Robot MUST Be
    // stationary. Device takes a large number of samples, and uses those
    // as the gyroscope zero-offset. This takes approximately 0.25 seconds.
    PinpointBlocks.recalibrateIMU();
    // Resets the current position to 0,0,0 and recalibrates the Odometry Computer's
    // internal IMU. Robot MUST Be stationary. Device takes a large number of samples,
    // and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
    PinpointBlocks.resetPosAndIMU();
    // Setting the Boolean to true reverses the encoder, false leaves it normal
    PinpointBlocks.reverseEncoders(true, true);
    // sets the number of ticks per mm of linear travel for the odometry pod you are using
    // ticks per unit of the goBILDA 4-Bar Odometry Pod
    PinpointBlocks.encoderResolution(PinpointBlocks.FourBarOdometryPod(DistanceUnit.INCH), DistanceUnit.INCH);
    // not exact
    // Sets the odometry pod positions relative to the point that the odometry computer
    // tracks around.The X pod offset refers to how far sideways from the tracking point
    // the X (forward) odometry pod is. left is positivethe Y Pod offset refers to how far
    // forward from the tracking point the Y (strafe) odometry pod is. Forward increases
    PinpointBlocks.offsets(DistanceUnit.INCH, -5.1, 1.6);
    Lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ((DcMotorEx) Lb).setPositionPIDFCoefficients(5);
    ((DcMotorEx) Lf).setPositionPIDFCoefficients(5);
    ((DcMotorEx) Rb).setPositionPIDFCoefficients(5);
    ((DcMotorEx) Rf).setPositionPIDFCoefficients(5);
    changeMotorPIDF(1.2, 1.2, 0, 12);
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
    imu.resetYaw();
  }

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    Lb = hardwareMap.get(DcMotor.class, "Lb");
    Lf = hardwareMap.get(DcMotor.class, "Lf");
    Rb = hardwareMap.get(DcMotor.class, "Rb");
    Rf = hardwareMap.get(DcMotor.class, "Rf");
    Llin = hardwareMap.get(DcMotor.class, "Llin");
    Rlin = hardwareMap.get(DcMotor.class, "Rlin");
    rotat = hardwareMap.get(DcMotor.class, "rotat");
    pickmeup = hardwareMap.get(DcMotor.class, "pickmeup");
    
    imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");
    ankel = hardwareMap.get(Servo.class, "ankel");
    
    Sensor = hardwareMap.get(DistanceSensor.class, "Sensor");
    
    imu = hardwareMap.get(IMU.class, "imu");

    initialize();
    waitForStart();
    if (opModeIsActive()) {
      forward(24, 0.6);
      IMUTurn(90, 0.2);
      strafe(12, 0.6);
      telemetry2();
    }
  }

  /**
   * Describe this function...
   */
  private void move(int speed) {
    Lb.setPower(-speed);
    Lf.setPower(-speed);
    Rb.setPower(speed);
    Rf.setPower(speed);
  }

  private void turn(int speed) {
    Lb.setPower(speed);
    Lf.setPower(-speed);
    Rb.setPower(speed);
    Rf.setPower(-speed);
  }
  
  /**
   * Describe this function...
   */
  private void strafing(int speed) {
    Lb.setPower(-speed);
    Lf.setPower(speed);
    Rb.setPower(-speed);
    Rf.setPower(speed);
  }

  /**
   * Describe this function...
   */
  private double getAngle() {
    // gets the angle of the robot
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }

  /**
   * Describe this function...
   */
  private void changeMotorPIDF(double p, double i, int d, int f) {
    ((DcMotorEx) Lb).setVelocityPIDFCoefficients(p, i, d, f);
    ((DcMotorEx) Lf).setVelocityPIDFCoefficients(p, i, d, f);
    ((DcMotorEx) Rb).setVelocityPIDFCoefficients(p, i, d, f);
    ((DcMotorEx) Rf).setVelocityPIDFCoefficients(p, i, d, f);
  }

  /**
   * Describe this function...
   */
  private void telemetry2() {
    // Returns x position the unit of your choice
    // Returns y position the unit of your choice
    telemetry.addData("Pose: ", "(" + Double.parseDouble(JavaUtil.formatNumber(PinpointBlocks.xPosition(DistanceUnit.INCH), 3)) + "," + Double.parseDouble(JavaUtil.formatNumber(PinpointBlocks.yPosition(DistanceUnit.INCH), 3)) + ")");
    telemetry.addLine("Forward is X positive, Left is Y positive");
    // Returns the direction your robot is facing the unit of your choice
    telemetry.addData("Orientation using Degrees: ", PinpointBlocks.orientation(AngleUnit.DEGREES));
    telemetry.addData("speed", tempSpeed);
    // Call this once per loop to read new data from the Odometry Computer. Data will only update once this is called.
    PinpointBlocks.update();
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void IMUTurn(int angle, double turnSpeed) {
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Angle from 0 to 180 is turning left from the 0 point, Angle from 0 to -180 is turning right from the 0 point
    max = angle + 0.5;
    min = angle - 0.5;
    if (getAngle() < min || getAngle() > max) {
      // while the angle is less than min, or the angle is more than max, and the op mode is active
      while ((getAngle() < min || getAngle() > max) && opModeIsActive()) {
        // if the current angle is less than min turn left, when the angle is greater than max, stop
        if (getAngle() < min) {
          // Positive is left, negative is right
          // turning with the set speed
          strafing((int) turnSpeed);
          // while the angle is less than min, or the angle is more than max, and the op mode is active
          while ((getAngle() < min || getAngle() > max) && opModeIsActive()) {
            if (getAngle() > max) {
              break;
            }
          }
          // stops turning
          strafing(0);
        } else if (getAngle() > max) {
          // Positive is left, negative is right
          // turning with the set speed
          strafing((int) -turnSpeed);
          // while the angle is less than min, or the angle is more than max, and the op mode is active
          while ((getAngle() < min || getAngle() > max) && opModeIsActive()) {
            if (getAngle() < min) {
              break;
            }
          }
          // stops turning
          strafing(0);
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private void forward(int dist, double speed) {
    tempSpeed = speed;
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Angle from 0 to 180 is turning left from the 0 point, Angle from 0 to -180 is turning right from the 0 point
    telemetry2();
    max = dist + 0.5;
    min = dist - 0.5;
    // while the angle is less than min, or the angle is more than max, and the op mode is active
    telemetry2();
    // Returns x position the unit of your choice
    // Returns x position the unit of your choice
    if (PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) {
      // Returns x position the unit of your choice
      // Returns x position the unit of your choice
      while ((PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
        telemetry2();
        // if the current angle is less than min turn left, when the angle is greater than max, stop
        // Returns x position the unit of your choice
        // Returns x position the unit of your choice
        if (PinpointBlocks.xPosition(DistanceUnit.INCH) < min) {
          // Positive is left, negative is right
          // turning with the set speed
          telemetry2();
          move((int) tempSpeed);
          // Returns x position the unit of your choice
          // Returns x position the unit of your choice
          while ((PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            // Returns x position the unit of your choice
            if (PinpointBlocks.xPosition(DistanceUnit.INCH) > max) {
              telemetry2();
              break;
            }
          }
          // stops turning
          move(0);
          telemetry2();
        } else if (PinpointBlocks.xPosition(DistanceUnit.INCH) > max) {
          // Positive is left, negative is right
          // turning with the set speed
          telemetry2();
          move((int) -tempSpeed);
          // while the angle is less than min, or the angle is more than max, and the op mode is active
          // Returns x position the unit of your choice
          // Returns x position the unit of your choice
          while ((PinpointBlocks.xPosition(DistanceUnit.INCH) < min || PinpointBlocks.xPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            // Returns x position the unit of your choice
            if (PinpointBlocks.xPosition(DistanceUnit.INCH) < min) {
              telemetry2();
              break;
            }
          }
          telemetry2();
          move(0);
        }
        tempSpeed = 0.1;
      }
    }
  }

private void TurnRight(double turns) {
  imu.resetYaw();
  yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
  Lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  Lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  Rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  Rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    while (yeeyaw > -50 * Math.pow(turns, 1.5)) {
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    Lb.setPower(-power * 1.2);
    Lf.setPower(-power * 1.2);
    Rb.setPower(-power * 1.2);
    Rf.setPower(-power * 1.2);
    telemetry.addLine("fast turn: " + yeeyaw);
    telemetry.update();
      }
      if (yeeyaw > -90 * turns) {
        while (yeeyaw > -90 * turns) {
        yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        Lb.setPower(-power * 0.4);
        Lf.setPower(-power * 0.4);
        Rb.setPower(-power * 0.4);
        Rf.setPower(-power * 0.4);
        telemetry.addLine("slow turn: " + yeeyaw);
        telemetry.update();
      }
    }
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    imu.resetYaw();
  }

  /**
   * Describe this function...
   */
  private void strafe(int dist, double speed) {
    tempSpeed = speed;
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // Angle from 0 to 180 is turning left from the 0 point, Angle from 0 to -180 is turning right from the 0 point
    telemetry2();
    max = dist + 0.5;
    min = dist - 0.5;
    // while the angle is less than min, or the angle is more than max, and the op mode is active
    telemetry2();
    // Returns y position the unit of your choice
    if (PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) {
      // Returns y position the unit of your choice
      while ((PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
        telemetry2();
        // if the current angle is less than min turn left, when the angle is greater than max, stop
        if (PinpointBlocks.yPosition(DistanceUnit.INCH) < min) {
          // Positive is left, negative is right
          // turning with the set speed
          telemetry2();
          strafing((int) tempSpeed);
          // Returns y position the unit of your choice
          while ((PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            // Returns y position the unit of your choice
            if (PinpointBlocks.yPosition(DistanceUnit.INCH) > max) {
              telemetry2();
              break;
            }
          }
          // stops turning
          move(0);
          telemetry2();
        } else if (PinpointBlocks.yPosition(DistanceUnit.INCH) > max) {
          // Positive is left, negative is right
          // turning with the set speed
          telemetry2();
          strafing((int) -tempSpeed);
          // while the angle is less than min, or the angle is more than max, and the op mode is active
          // Returns y position the unit of your choice
          // Returns y position the unit of your choice
          while ((PinpointBlocks.yPosition(DistanceUnit.INCH) < min || PinpointBlocks.yPosition(DistanceUnit.INCH) > max) && opModeIsActive()) {
            telemetry2();
            // Returns y position the unit of your choice
            if (PinpointBlocks.yPosition(DistanceUnit.INCH) < min) {
              telemetry2();
              break;
            }
          }
          telemetry2();
          move(0);
        }
        tempSpeed = 0.1;
      }
    }
  }
}
