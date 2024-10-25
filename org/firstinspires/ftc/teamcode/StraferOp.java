package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "StraferOpV3")
public class StraferOpV3 extends LinearOpMode {

  private DcMotor Lf;
  private DcMotor Rf;
  private DcMotor Lb;
  private DcMotor Rb;
  private DcMotor pickMeUp;
  private DcMotor Llin;
  private DcMotor Rlin;
  private Servo imaTouchU;
  
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
    pickMeUp = hardwareMap.get(DcMotor.class, "pickmeup");
    Llin = hardwareMap.get(DcMotor.class, "Llin");
    Rlin = hardwareMap.get(DcMotor.class, "Rlin");
    imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");
    
    imu = hardwareMap.get(IMU.class, "imu");
    
    Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    pickMeUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Llin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rlin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    Lf.setDirection(DcMotor.Direction.FORWARD);
    Rf.setDirection(DcMotor.Direction.REVERSE);
    Lb.setDirection(DcMotor.Direction.FORWARD);
    Rb.setDirection(DcMotor.Direction.REVERSE);
    pickMeUp.setDirection(DcMotor.Direction.REVERSE);
    Llin.setDirection(DcMotor.Direction.REVERSE);
    Rlin.setDirection(DcMotor.Direction.FORWARD);
    
    //pickMeUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    pickMeUp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Llin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rlin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    imaTouchU.scaleRange(.2, .8);
    
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
      
      if (gamepad2.right_trigger > 0.1){
        liftSystem(7400);
        Llin.setPower(1);
        Rlin.setPower(1);
      } else if (gamepad2.left_trigger > 0.1){
        liftSystem(0);
        Llin.setPower(1);
        Rlin.setPower(1);
      } else {
        Llin.setPower(0);
        Rlin.setPower(0);
      }
      
      if (gamepad2.right_bumper){
        //pickMeUp.setPower(.3);
        extendoGrip(1020);
      } else if (gamepad2.left_bumper){
        //pickMeUp.setPower(-.1);
        extendoGrip(30);
      }
      
      if (gamepad2.x){
        imaTouchU.setPosition(.16);
      } else if (gamepad2.b){
        imaTouchU.setPosition(.5);
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
  
  public void extendoGrip(int pos) {
    pickMeUp.setTargetPosition(pos);
    pickMeUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickMeUp.setPower(1);
  }
  
  public void liftSystem(int pos){
    Llin.setTargetPosition(pos);
    Rlin.setTargetPosition(pos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
