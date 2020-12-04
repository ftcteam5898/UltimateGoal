package org.firstinspires.ftc.teamcode;

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
      double r = Math.hypot((this.gamepad1.left_stick_x - this.gamepad1.right_stick_x), (this.gamepad1.left_stick_y - this.gamepad1.right_stick_y));
      double robotAngle = Math.atan2((this.gamepad1.left_stick_y - this.gamepad1.right_stick_y), (-this.gamepad1.left_stick_x + this.gamepad1.right_stick_x)) - 0.7853981633974483D;
      double rightX = (this.gamepad1.right_trigger - this.gamepad1.left_trigger);
      double v1 = r * Math.cos(robotAngle) - rightX;
      double v2 = r * Math.sin(robotAngle) + rightX;
      double v3 = r * Math.sin(robotAngle) - rightX;
      double v4 = r * Math.cos(robotAngle) + rightX;
      this.lf.setPower(v1);
      this.rf.setPower(-v2);
      this.lb.setPower(v3);
      this.rb.setPower(-v4);
      if (this.gamepad2.b) {
        this.arm.setPower(1.0D);
      } else {
        this.arm.setPower(0.0D);
      } 
      if (this.gamepad2.a) {
        this.arm.setPower(-1.0D);
      } else {
        this.arm.setPower(0.0D);
      } 
      if (this.gamepad2.x) {
        this.grab.setPower(0.2D);
        continue;
      } 
      if (this.gamepad2.y) {
        this.grab.setPower(-0.2D);
        continue;
      } 
      this.grab.setPower(0.0D);
    } 
  }
  
  public void setupHardware() {
    this.lb = (DcMotor)this.hardwareMap.get(DcMotor.class, "lb");
    this.lf = (DcMotor)this.hardwareMap.get(DcMotor.class, "lf");
    this.rb = (DcMotor)this.hardwareMap.get(DcMotor.class, "rb");
    this.rf = (DcMotor)this.hardwareMap.get(DcMotor.class, "rf");
    this.arm = (DcMotor)this.hardwareMap.get(DcMotor.class, "arm");
    this.grab = (CRServo)this.hardwareMap.get(CRServo.class, "grab");
  }
}