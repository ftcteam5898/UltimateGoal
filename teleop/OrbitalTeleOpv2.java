package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "OrbitalTeleOpv2", group = "TeleOp")
public class OrbitalTeleOpv2 extends LinearOpMode {
  private DcMotor lb;
  
  private DcMotor lf;
  
  private DcMotor rb;
  
  private DcMotor rf;
  
  private DcMotor arm;
  
  private CRServo grab;
  
  private double grabPos = 0.0D;
  
  public void runOpMode() {
    setupHardware();
    waitForStart();
    while (opModeIsActive()) {
      double r = Math.hypot((gamepad1.left_stick_x - gamepad1.right_stick_x), (gamepad1.left_stick_y - gamepad1.right_stick_y));
      double robotAngle = Math.atan2((gamepad1.left_stick_y - gamepad1.right_stick_y), (-gamepad1.left_stick_x + gamepad1.right_stick_x)) - 0.7853981633974483D;
      double rightX = (gamepad1.right_trigger - gamepad1.left_trigger);
      double v1 = r * Math.cos(robotAngle) - rightX;
      double v2 = r * Math.sin(robotAngle) + rightX;
      double v3 = r * Math.sin(robotAngle) - rightX;
      double v4 = r * Math.cos(robotAngle) + rightX;
      lf.setPower(v1);
      rf.setPower(-v2);
      lb.setPower(v3);
      rb.setPower(-v4);
      if (gamepad2.b) {
        arm.setPower(1.0D);
      } else {
        arm.setPower(0.0D);
      } 
      if (gamepad2.a) {
        arm.setPower(-1.0D);
      } else {
        arm.setPower(0.0D);
      } 
      if (gamepad2.x) {
        grab.setPower(0.2D);
        continue;
      } 
      if (gamepad2.y) {
        grab.setPower(-0.2D);
        continue;
      } 
      grab.setPower(0.0D);
    } 
  }
  
  public void setupHardware() {
    lb = hardwareMap.get(DcMotor.class, "lb");
    lf = hardwareMap.get(DcMotor.class, "lf");
    rb = hardwareMap.get(DcMotor.class, "rb");
    rf = hardwareMap.get(DcMotor.class, "rf");
    arm = hardwareMap.get(DcMotor.class, "arm");
    grab = hardwareMap.get(CRServo.class, "grab");
  }
}