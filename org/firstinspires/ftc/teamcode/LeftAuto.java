package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "LeftAuto", preselectTeleOp = "StraferOpV3")
public class LeftAuto extends LinearOpMode { 
  
  private DcMotor Lf;
  private DcMotor Rf;
  private DcMotor Lb;
  private DcMotor Rb;
  private DcMotor pickmeup;
  private DcMotor Llin;
  private DcMotor Rlin;
  private DcMotor rotat;
  private IMU imu;
  private DistanceSensor Sensor;
  
  private Servo imaTouchU;
  private Servo ankel;
  
  double IMUpower = .5;
  double armSpeed = .9;
  double power = .4;
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
    Lb.setTargetPosition(Lb.getCurrentPosition() - _targetPos);
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
  Lb.setTargetPosition(Lb.getCurrentPosition() - _targetPos);
  Lf.setTargetPosition(Lf.getCurrentPosition() + _targetPos);
  Rb.setTargetPosition(Rb.getCurrentPosition() - _targetPos);
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
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    Lb.setTargetPosition(Lb.getCurrentPosition() + _targetPos - 60);
    Lf.setTargetPosition(Lf.getCurrentPosition() - _targetPos);
    Rb.setTargetPosition(Rb.getCurrentPosition() + _targetPos - 60);
    Rf.setTargetPosition(Rf.getCurrentPosition() - _targetPos);
    Lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lb.setPower(power - yeeyaw / 15);
    Lf.setPower(power + yeeyaw / 15);
    Rb.setPower(power - yeeyaw / 15);
    Rf.setPower(power + yeeyaw / 15);
    while (Lb.isBusy()) {
      yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      Lb.setPower(power - yeeyaw / 15);
      Lf.setPower(power + yeeyaw / 15);
      Rb.setPower(power - yeeyaw / 15);
      Rf.setPower(power + yeeyaw / 15);
      telemetry.addData("Angle: ", yeeyaw);
      telemetry.update();
    }
  }

  
  private void TurnRightC(int _targetPos) {
    Lb.setTargetPosition(Lb.getCurrentPosition() - _targetPos); 
    Lf.setTargetPosition(Lf.getCurrentPosition() - _targetPos);
    Rb.setTargetPosition(Rb.getCurrentPosition() - _targetPos);
    Rf.setTargetPosition(Rf.getCurrentPosition() - _targetPos);
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
  
    private void TurnLeftC(int _targetPos) {
    Lb.setTargetPosition(Lb.getCurrentPosition() + _targetPos); 
    Lf.setTargetPosition(Lf.getCurrentPosition() + _targetPos);
    Rb.setTargetPosition(Rb.getCurrentPosition() + _targetPos);
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
  
  private void WallForward(double dist) {
    Lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      while (Sensor.getDistance(DistanceUnit.CM) > dist){
        Lb.setPower(-.25);
        Lf.setPower(.25);
        Rb.setPower(-.25);
        Rf.setPower(.25);
        telemetry.addData("Sensor", Sensor.getDeviceName() );
        telemetry.addData("Distance (cm)", Sensor.getDistance(DistanceUnit.CM));
        telemetry.update();
      } 
          Lb.setPower(0);
          Lf.setPower(0);
          Rb.setPower(0);
          Rf.setPower(0);
          Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  
  private void ArmIn(int _targetPos) {
    pickmeup.setTargetPosition(pickmeup.getCurrentPosition() + _targetPos);
    pickmeup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickmeup.setPower(power);
    while (pickmeup.isBusy()) {
    }
  }
  
  private void ArmOut(int _targetPos) {
    pickmeup.setTargetPosition(pickmeup.getCurrentPosition() - _targetPos);
    pickmeup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickmeup.setPower(power);
    while (pickmeup.isBusy()) {
    }
  }
  
  private void ArmUp(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() + _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
    while (rotat.isBusy()) {
    }
  }
  
  private void ArmDown(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() - _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
    while (rotat.isBusy()) {
    }
  }
  
  private void SimulArmUp(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() + _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
  }
  
  private void SimulArmDown(int _targetPos) {
    rotat.setTargetPosition(rotat.getCurrentPosition() - _targetPos);
    rotat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rotat.setPower(armSpeed);
  }
  
  private void SlidesUp(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() - _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() + _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
    while (Llin.isBusy()) {
    }
  }
  
  private void SlidesDown(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() + _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() - _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
    while (Llin.isBusy()) {
    }
  }
  
  private void SimulSlidesUp(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() - _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() + _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
  }
  
  private void SimulSlidesDown(int _targetPos) {
    Llin.setTargetPosition(Llin.getCurrentPosition() + _targetPos);
    Rlin.setTargetPosition(Rlin.getCurrentPosition() - _targetPos);
    Llin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rlin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Llin.setPower(armSpeed);
    Rlin.setPower(armSpeed);
  }
  
  private void ClampClaw(double clamp_sp, int sleeptime) {
    imaTouchU.setPosition(clamp_sp);
    sleep(sleeptime);
  }
  
  private void MoveClaw(double claw_sp, int sleeptime) {
    ankel.setPosition(claw_sp);
    sleep(sleeptime);
  }

  private void TurnLeft(double turns) {
    imu.resetYaw();
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    Lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  while (yeeyaw < 50 * Math.pow(turns, 1.5)) {
    yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    Lb.setPower(-power * -1.2);
    Lf.setPower(-power * -1.2);
    Rb.setPower(-power * -1.2);
    Rf.setPower(-power * -1.2);
    telemetry.addLine("fast turn: " + yeeyaw);
    telemetry.update();
  }  
  if (yeeyaw < 90 * turns) {
    while (yeeyaw < 90 * turns) {
      yeeyaw = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      Lb.setPower(-power * -0.4);
      Lf.setPower(-power * -0.4);
      Rb.setPower(-power * -0.4);
      Rf.setPower(-power * -0.4);
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
  
   @Override
  public void runOpMode() {
    int turnSpeed;
    int currentPos;
    int minRange;
    int maxRange;
    int armSpeed;
    
    Rb = hardwareMap.get(DcMotor.class, "Rb");
    Rf = hardwareMap.get(DcMotor.class, "Rf");
    Lb = hardwareMap.get(DcMotor.class, "Lb");
    Lf = hardwareMap.get(DcMotor.class, "Lf");
    Llin = hardwareMap.get(DcMotor.class, "Llin");
    Rlin = hardwareMap.get(DcMotor.class, "Rlin");
    rotat = hardwareMap.get(DcMotor.class, "rotat");
    pickmeup = hardwareMap.get(DcMotor.class, "pickmeup");
    
    imaTouchU = hardwareMap.get(Servo.class, "imaTouchU");
    ankel = hardwareMap.get(Servo.class, "ankel");
    
    Sensor = hardwareMap.get(DistanceSensor.class, "Sensor");
    
    imu = hardwareMap.get(IMU.class, "imu");
    
    Lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Llin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Rlin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rotat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Llin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rlin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rotat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    currentPos = 0;
    turnSpeed = 818;
    minRange = 6;
    maxRange = 28;
    imaTouchU.scaleRange(.2, .8);
    ankel.scaleRange(0, 1);
    telemetry.addData("Sensor", Sensor.getDeviceName() );
    telemetry.update();
    

    waitForStart();

  while (opModeIsActive()) {
    telemetry.addData("Distance (cm)", Sensor.getDistance(DistanceUnit.CM));
    telemetry.update();
    ClampClaw(0.16, 0);
    SimulArmUp(1250);
    SlidesUp(4400);
    SimulSlidesUp(2200);
    MoveClaw(.592, 0);
    ArmUp(500);
    Forward(890);
    ClampClaw(.5, 300);
    sleep(700);
    Forward(-600);
    power = .27;
    sleep(150);
    Right(170);
    power = .4;
    TurnRight(.9806);
    sleep(125);
    Left(271);
    ArmDown(1200);
    SlidesDown(6250);
    sleep(300);
    Forward(756);
    WallForward(16); 
    ArmDown(350);
    ClampClaw(.7, 0);
    MoveClaw(.567, 0);
    ArmOut(805);
    ClampClaw(.16, 600);
    ArmIn(705);
    ArmUp(1250);
    SlidesUp(6500);
    MoveClaw(.592, 0);
    ArmUp(250); 
    TurnLeft(1.5);
    Forward(720);
    sleep(400);
    ClampClaw(.5, 0);
    sleep(400);
    Forward(-400); 
    break; 
    }
  
  } 
    
}
