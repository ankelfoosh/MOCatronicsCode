/*
 * Adding a comment to test Github.dev web IDE
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name = "StraferOpV1")
public class StraferOpV1 extends LinearOpMode {

  private DcMotor Lf;
  private DcMotor Rf;
  private DcMotor Lb;
  private DcMotor Rb;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double RobotSpeed;
    double TurnSpeed;
    
    Lf = hardwareMap.get(DcMotor.class, "Lf");
    Rf = hardwareMap.get(DcMotor.class, "Rf");
    Lb = hardwareMap.get(DcMotor.class, "Lb");
    Rb = hardwareMap.get(DcMotor.class, "Rb");
    
    Lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    Lf.setDirection(DcMotor.Direction.REVERSE);
    Rf.setDirection(DcMotor.Direction.FORWARD);
    Lb.setDirection(DcMotor.Direction.REVERSE);
    Rb.setDirection(DcMotor.Direction.FORWARD);
    
    waitForStart();
    while (opModeIsActive()) {
      // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
      // We negate this value so that the topmost position corresponds to maximum forward power.
      Lf.setPower(TurnSpeed * gamepad1.right_stick_x * RobotSpeed + RobotSpeed * gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
      Rf.setPower(TurnSpeed * -gamepad1.right_stick_x * RobotSpeed + RobotSpeed * -gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
      // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
      // We negate this value so that the topmost position corresponds to maximum forward power.
      Lb.setPower(TurnSpeed * gamepad1.right_stick_x * RobotSpeed + RobotSpeed * -gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
      Rb.setPower(TurnSpeed * -gamepad1.right_stick_x * RobotSpeed + RobotSpeed * gamepad1.left_stick_x + RobotSpeed * gamepad1.left_stick_y);
      
      // When pressing right trigger, the robot goes into turbo mode.
      // When pressing left trigger, the robot goes into snail mode.
      if (gamepad1.right_trigger > 0.1) {
        RobotSpeed = 0.75;
        TurnSpeed = 0.85;
      } else if (gamepad1.left_trigger > 0.1) {
        RobotSpeed = 0.2;
        TurnSpeed = 1;
      } else {
        RobotSpeed = 0.42;
        TurnSpeed = 1.6;
      }
    }
  }
}
