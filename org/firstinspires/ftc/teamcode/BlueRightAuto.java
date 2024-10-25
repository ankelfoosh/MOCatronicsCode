package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "BlueRightAuto")
public class BlueRightAuto extends LinearOpMode {
  
  private DcMotor Lf;
  private DcMotor Rf;
  private DcMotor Lb;
  private DcMotor Rb;
  private IMU imu;
  private 
  
  double power = .45;
  float yeeyaw;

    public void initialize() { 
   Rf.setDirection(DcMotor.Direction.FORWARD);
   Rb.setDirection(DcMotor.Direction.FORWARD);
   Lb.setDirection(DcMotor.Direction.FORWARD);
   Lf.setDirection(DcMotor.Direction.FORWARD);
   Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   Rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   Lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   imu.resetYaw();
   
    }
  private void Forward(int _targetPos) {
    Rb.setTargetPosition(Rb.getCurrentPosition() + _targetPos);
    Rf.setTargetPosition(Rf.getCurrentPosition() + _targetPos);
    Lb.setTargetPosition(Lb.getCurrentPosition() -_targetPos);
    Lf.setTargetPosition(Lf.getCurrentPosition() - _targetPos);
    Lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lb.setPower(power);
    Lf.setPower(power);
    Rb.setPower(power);
    Rf.setPower(power);
    while (Lb.isBusy()) {
    
    telemetry.addData("RbCurrentPosition",Rb.getTargetPosition());
    telemetry.update();
    }
  }

// Stops and resets the encoder value stored in the motor


  private void Left(int _targetPos) {
  Lb.setTargetPosition(Lb.getCurrentPosition() + -_targetPos);
  Lf.setTargetPosition(Lf.getCurrentPosition() + _targetPos);
  Rb.setTargetPosition(Rb.getCurrentPosition() + -_targetPos);
  Rf.setTargetPosition(Rf.getCurrentPosition() + _targetPos);
  Lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  Lb.setPower(power);
    Lf.setPower(power);
    Rb.setPower(power);
    Rf.setPower(power);
  while (Lb.isBusy()) {
    telemetry.addData("RbCurrentPosition",Rb.getTargetPosition());
    telemetry.update();
  }
  }
  
  
  private void Right(int _targetPos) {
  Lb.setTargetPosition(Lb.getCurrentPosition() + _targetPos);
    Lf.setTargetPosition(Lf.getCurrentPosition() + -_targetPos);
    Rb.setTargetPosition(Rb.getCurrentPosition() + _targetPos);
    Rf.setTargetPosition(Rf.getCurrentPosition() + -_targetPos);
    Lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lb.setPower(power);
    Lf.setPower(power);
    Rb.setPower(power);
    Rf.setPower(power);
    while (Lb.isBusy()) {
    telemetry.addData("RbCurrentPosition",Rb.getTargetPosition());
    telemetry.update();
    }
  }
  
  private void TurnLeft(int turns) {
  imu.resetYaw();
  yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
  Lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  while (yeeyaw < 90 * turns) {
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
  Lb.setPower(-power * -1.2);
  Lf.setPower(-power * -1.2);
  Rb.setPower(-power * -1.2);
  Rf.setPower(-power * -1.2);
  telemetry.addLine("yeeyaw");
  telemetry.update();
  }  
  if (yeeyaw > 90 * turns) {
    while (yeeyaw > 90 * turns) {
      yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    Lb.setPower(power * -0.4);
    Lf.setPower(power * -0.4);
    Rb.setPower(power * -0.4);
    Rf.setPower(power * -0.4);
    telemetry.addLine("yeeyaw");
    telemetry.update();
   }
  }
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
    private void TurnRight(int turns) {
    imu.resetYaw();
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    Lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    while (yeeyaw > -90 * turns) {
      yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      Lb.setPower(-power * 1.2);
      Lf.setPower(-power * 1.2);
      Rb.setPower(-power * 1.2);
      Rf.setPower(-power * 1.2);
      telemetry.addLine("yeeyaw");
      telemetry.update();
    }
    if (yeeyaw < -90 * turns) {
      while (yeeyaw < -90 * turns) {
        yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        Lb.setPower(power * 0.4);
        Lf.setPower(power * 0.4);
        Rb.setPower(power * 0.4);
        Rf.setPower(power * 0.4);
        telemetry.addLine("yeeyaw");
        telemetry.update();
      }
    }
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
   @Override
  public void runOpMode() {
   
    int turnSpeed;
    int currentPos;
    
    Rb = hardwareMap.get(DcMotor.class, "Rb");
    Rf = hardwareMap.get(DcMotor.class, "Rf");
    Lb = hardwareMap.get(DcMotor.class, "Lb");
    Lf = hardwareMap.get(DcMotor.class, "Lf");
    imu = hardwareMap.get(IMU.class, "imu");
    Lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();

while (opModeIsActive()) {
    currentPos = 0;
    turnSpeed = 818;
    Forward(500);
    Right(900);
    Forward(1700);
    TurnRight(1);
    TurnRight(1);
    Left(500);
    Forward(1900);
    Right(400);
    sleep(200);
    Forward(-1800);
    Left(800);
    Forward(2100);
    Right(350);
    Forward(-2100);
    TurnLeft(1);
    Right(500);
    power = .25;
    Forward(600);
    sleep(700);
    Right(1400);
    break;
    }
  }
}
