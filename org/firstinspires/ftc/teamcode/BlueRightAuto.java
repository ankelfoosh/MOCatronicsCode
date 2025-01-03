package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;


@Autonomous(name = "BlueRightAuto", preselectTeleOp = "StraferOpV3")
public class BlueRightAuto extends LinearOpMode { 
  
  private DcMotor Lf;
  private DcMotor Rf;
  private DcMotor Lb;
  private DcMotor Rb;
  private DcMotor pickmeup;
  private DcMotor Llin;
  private DcMotor Rlin;
  private DcMotor rotat;
  private IMU imu;
  private DistanceSensor sensor;
  
  private Servo imaTouchU;
  private Servo ankel;
  
  double armSpeed = 1.0;
  double power = .6;
  float yeeyaw;
  
  double lbSpeed = 0.4;
  double rbSpeed = -0.7;
  double lfSpeed = 0.4;
  double rfSpeed = -0.7;
  
  int lbPos = 1000;
  int rbPos = 1750;
  int lfPos = -1000;
  int rfPos = -1750;
  
  
   
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
  
  private void WallForward(double dist) {
    while (sensor.getDistance(DistanceUnit.CM) > dist + 15){
      Lb.setPower(-power);
      Lf.setPower(-power);
      Rb.setPower(power);
      Rf.setPower(power);
      telemetry.addData("Sensor", sensor.getDeviceName() );
      telemetry.addData("Distance (cm)", sensor.getDistance(DistanceUnit.CM));
      telemetry.update();
    }
    
    while (sensor.getDistance(DistanceUnit.CM) > dist){
      Lb.setPower(-power * .3);
      Lf.setPower(-power * .3);
      Rb.setPower(power * .3);
      Rf.setPower(power * .3);
      telemetry.addData("Sensor", sensor.getDeviceName() );
      telemetry.addData("Distance (cm)", sensor.getDistance(DistanceUnit.CM));
      telemetry.update();
    }
    
    Lb.setPower(0);
    Lf.setPower(0);
    Rb.setPower(0);
    Rf.setPower(0);
  }

// Stops and resets the encoder value stored in the motor

  private void CoolStrafe(int lbtp, int rbtp, int lftp, int rftp){
    Lb.setTargetPosition(Lb.getCurrentPosition() + lbtp);
    Rb.setTargetPosition(Lf.getCurrentPosition() + rbtp);
    Lf.setTargetPosition(Rb.getCurrentPosition() + lftp);
    Rf.setTargetPosition(Rf.getCurrentPosition() + rftp);
    Lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    double average = (Math.abs(lbtp) + Math.abs(rbtp) + Math.abs(lftp) + Math.abs(rftp)) / 4;
    double powMult = average / (average * power);
    Lb.setPower(power * (average / Math.abs(lbtp)));
    Rb.setPower(power * (average / Math.abs(rbtp)));
    Lf.setPower(power * (average / Math.abs(lftp)));
    Rf.setPower(power * (average / Math.abs(rftp)));
    while (Lb.isBusy()) {
    }
  }

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
    Lb.setTargetPosition(Lb.getCurrentPosition() + _targetPos - 390);
    Lf.setTargetPosition(Lf.getCurrentPosition() - _targetPos);
    Rb.setTargetPosition(Rb.getCurrentPosition() + _targetPos - 390);
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
  
  private void ArmIn(int _targetPos) {
    pickmeup.setTargetPosition(pickmeup.getCurrentPosition() + _targetPos);
    pickmeup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickmeup.setPower(armSpeed);
    while (pickmeup.isBusy()) {
    }
  }
  
  private void ArmOut(int _targetPos) {
    pickmeup.setTargetPosition(pickmeup.getCurrentPosition() - _targetPos);
    pickmeup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pickmeup.setPower(armSpeed);
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
  
  private void CloseClaw() {
    imaTouchU.setPosition(0.16);
    sleep(300);
  }
    
  private void OpenClaw() {
    imaTouchU.setPosition(0.58);
    sleep(300);
  }
  
  private void OpenClaw2() {
    imaTouchU.setPosition(0.6);
    sleep(500);
  }
  
  private void ClawDown() {
    ankel.setPosition(.567);
  }
  
  private void ClawUp() {
    ankel.setPosition(.658);
  }
  
  private void ClawSet(){
    ankel.setPosition(.612);
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
        Lb.setPower(-power * 0.3);
        Lf.setPower(-power * 0.3);
        Rb.setPower(-power * 0.3);
        Rf.setPower(-power * 0.3);
        telemetry.addLine("slow turn: " + yeeyaw);
        telemetry.update();
      }
    }
    Lb.setPower(0);
    Lf.setPower(0);
    Rb.setPower(0);
    Rf.setPower(0);
    Lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    imu.resetYaw();
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
      Lb.setPower(-power * -0.3);
      Lf.setPower(-power * -0.3);
      Rb.setPower(-power * -0.3);
      Rf.setPower(-power * -0.3);
      telemetry.addLine("slow turn: " + yeeyaw);
      telemetry.update();
    }
  }
    Lb.setPower(0);
    Lf.setPower(0);
    Rb.setPower(0);
    Rf.setPower(0);
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

 /* IMU.Parameters myIMUparameters;
  myIMUparameters parameters = new IMU.Parameters(
  new RevHubOrientationOnRobot (
          RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
          RevHubOrientationOnRobot.UsbFacingDirection.DOWN
      )
  ); 
  parameters.mode               = BNO055IMU.SensorMode.IMU;
  parameters.angleUnit          = BNO055IMU.AngleUnit.DEGREES;
  parameters.accelUnit          = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; */
    
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
    
    sensor = hardwareMap.get(DistanceSensor.class, "Sensor");
    
    imu = hardwareMap.get(IMU.class, "imu");
    
    Rf.setDirection(DcMotor.Direction.FORWARD);
    Rb.setDirection(DcMotor.Direction.FORWARD);
    Lb.setDirection(DcMotor.Direction.FORWARD);
    Lf.setDirection(DcMotor.Direction.FORWARD);
    Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    imu.resetYaw();
    
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
    minRange = 3;
    maxRange = 14;
    imaTouchU.scaleRange(.2, .8);
    ankel.scaleRange(0, 1);
    telemetry.addData("Sensor", sensor.getDeviceName() );
    telemetry.update();
     
    
//    imu.initialize(parameters);

    waitForStart();

while (opModeIsActive()) {
    imu.resetYaw();
    CloseClaw();
    SimulArmUp(670);
    SimulSlidesUp(2000);
    ClawUp();
    Forward(1180);
    ArmOut(1020);
    ArmIn(990);
    OpenClaw();
    sleep(70);
    Forward(-540);
    SimulArmDown(550);
    SimulSlidesDown(1620);
    sleep(200);
    Right(2222);
    //TurnRight(0.98);
    //Forward(1900);
    //TurnLeft(1);
    // power = 0.2;
    // Forward(30);
    // power = 0.6;
    // ArmOut(1020);
    // ClawDown();
    // sleep(300);
    // CloseClaw();
    // sleep(200);
    SimulArmUp(120);
    //ArmIn(990);
    TurnLeft(1.975);
    ClawSet();
    WallForward(30);
    OpenClaw();
    power = 0.2;
    Forward(195);
    sleep(300);
    CloseClaw();
    sleep(200);
    power = 0.6;
    //armSpeed = 0.4;
    SimulArmUp(420);
    SimulSlidesUp(1800);
    Forward(-200);
    ClawUp();
    //armSpeed = 1;
    Right(2700);
    TurnRight(1.975);
    Forward(760);
    ArmOut(1020);
    ArmIn(990);
    OpenClaw();
    sleep(70);
    Forward(-400);
    SimulArmDown(310);
    SimulSlidesDown(1670);
    Forward(-500);
    sleep(200);
    imu.resetYaw();
    Right(2700);
    
    ClawSet();
    // TurnLeft(1.975);
    // Forward(120);
    // sleep(300);
    // CloseClaw();
    // sleep(200);
    // imu.resetYaw();
    // ClawUp();
    // SimulArmUp(360);
    // SimulSlidesUp(2000);
    // Forward(-200);
    // Right(2850);
    // TurnRight(1.975);
    // Forward(600);
    // ArmOut(1020);
    // ArmIn(990);
    // OpenClaw();
    // sleep(70);
    // Forward(-800);
    // Right(2850);
    //WallForward(30);
    // sleep(200);
    // CloseClaw();
    // sleep(200);
    // SimulArmUp(310);
    // SimulSlidesUp(1500);
    // Forward(-650);
    // ClawUp();
    // TurnRight(1.02);
    // TurnRight(1.02);
    // Right(-2050);
    // WallForward(12);
    // ArmOut(900);
    // ArmIn(870);
    // OpenClaw();
    // sleep(70);
    // Forward(-400);
    
    //CoolStrafe(6000, 6000, 1500, 1500);
    /*
    Forward(385);
    sleep(200);
    OpenClaw();
    sleep(166);
    Forward(-200);
    sleep(130);
    power = .86;
    SlidesDown(3900); 
    sleep(85);
    power = .69;
    TurnRightC(1200);
    sleep(100);
    Right(558);
    ArmDown(1310);
    Forward(790);
    ClawDown();
    OpenClaw2();
    if (Sensor.getDistance(DistanceUnit.CM) > minRange && Sensor.getDistance(DistanceUnit.CM) < maxRange) {
      ArmOut(1100);
      CloseClaw();
      sleep(100);
      ArmUp(1320);
       break;
    }  */
      break;
    }
  }
}
