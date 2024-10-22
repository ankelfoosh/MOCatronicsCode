package org.firstinspires.ftc.teamcode;

/*
 * This is strafer code
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "StraferOpV2")
public class StraferOpV2 extends LinearOpMode {

  private DcMotor Lf;
  private DcMotor Rf;
  private DcMotor Lb;
  private DcMotor Rb;
  
  private IMU imu;
  
  private double RobotSpeed = .42;
  private double TurnSpeed = 1.6;
  private boolean bigTurn = false;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    Lf = hardwareMap.get(DcMotor.class, "Lf");
    Rf = hardwareMap.get(DcMotor.class, "Rf");
    Lb = hardwareMap.get(DcMotor.class, "Lb");
    Rb = hardwareMap.get(DcMotor.class, "Rb");
    
    imu = hardwareMap.get(IMU.class, "imu");
    
    Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    Lf.setDirection(DcMotor.Direction.FORWARD);
    Rf.setDirection(DcMotor.Direction.FORWARD);
    Lb.setDirection(DcMotor.Direction.FORWARD);
    Rb.setDirection(DcMotor.Direction.REVERSE);
    
    waitForStart();
    while (opModeIsActive()) {
      if (!bigTurn) {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        Lf.setPower(TurnSpeed * -gamepad1.right_stick_x * RobotSpeed + RobotSpeed * -gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
        Rf.setPower(TurnSpeed * gamepad1.right_stick_x * RobotSpeed + RobotSpeed * gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        Lb.setPower(TurnSpeed * -gamepad1.right_stick_x * RobotSpeed + RobotSpeed * gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
        Rb.setPower(TurnSpeed * gamepad1.right_stick_x * RobotSpeed + RobotSpeed * -gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
      }
      
      // When pressing right trigger, the robot goes into turbo mode.
      // When pressing left trigger, the robot goes into snail mode.
      if (gamepad1.right_trigger > 0.1) {
        RobotSpeed = 1;
        TurnSpeed = 0.85;
      }
      else if (gamepad1.left_trigger > 0.1) {
        RobotSpeed = 0.2;
        TurnSpeed = 1;
      }
      else {
        RobotSpeed = 0.42;
        TurnSpeed = 1.6;
      }
      
      if (gamepad1.a){
        telemetry.addData("Yaw", getAngle());
        telemetry.update();
      }
      
      if (gamepad1.right_bumper) {
        if (gamepad1.x) {
          RightATW(90);
        } else {
          RightIMUTurn(90);
        }
        telemetry.addData("Yaw", getAngle());
        telemetry.update();
      }
      if (gamepad1.left_bumper) {
        if (gamepad1.x) {
          LeftATW(90);
        } else {
          LeftIMUTurn(90);
        }
        telemetry.addData("Yaw", getAngle());
        telemetry.update();
      }
    }
  }
    
  public void RightIMUTurn(double degree) {
    imu.resetYaw();
    double togNum = getAngle() - (degree - 20);
    double angularTurn = 1;
    boolean enlarge = false;
      
    while (getAngle() > togNum) {
      if (gamepad1.a && !enlarge){
        togNum -= 90;
        enlarge = true;
      }
      Lf.setPower(-angularTurn);
      Rf.setPower(angularTurn);
      Lb.setPower(-angularTurn);
      Rb.setPower(angularTurn);
    }
  }
  
  public void LeftIMUTurn(double degree) {
    imu.resetYaw();
    double togNum = getAngle() + (degree - 20);
    double angularTurn = 1;
    boolean enlarge = false;
      
    while (getAngle() < togNum) {
      if (gamepad1.a && !enlarge){
        togNum += 90;
        enlarge = true;
      }
      Lf.setPower(angularTurn);
      Rf.setPower(-angularTurn);
      Lb.setPower(angularTurn);
      Rb.setPower(-angularTurn);
    }
  }
  
  public void RightATW(double degree) {
    imu.resetYaw();
    double togNum = getAngle() + (degree - 10);
    double angularTurn = 1;
    
    while (getAngle() < togNum) {
      Lf.setPower(-0.08 * angularTurn);
      Rf.setPower(0.08 * angularTurn);
      Lb.setPower(angularTurn);
      Rb.setPower(-angularTurn);
    }
  }
  
  public void LeftATW(double degree) {
    imu.resetYaw();
    double togNum = getAngle() - (degree - 10);
    double angularTurn = 1;
    
    while (getAngle() > togNum) {
      Lf.setPower(0.08 * angularTurn);
      Rf.setPower(-0.08 * angularTurn);
      Lb.setPower(-angularTurn);
      Rb.setPower(angularTurn);
    }
  }
  
  private double getAngle() {
    return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
  }
}
