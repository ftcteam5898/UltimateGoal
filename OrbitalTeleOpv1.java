package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "OrbitalTeleOpv1", group = "TeleOp")
public class OrbitalTeleOpv1 extends LinearOpMode {
  private DcMotor lb;
  
  private DcMotor lf;
  
  private DcMotor rb;
  
  private DcMotor rf;
  
  private DcMotor in;
  
  private DcMotor lift;
  
  private CRServo dump;
  
  private int lp = 0;
  
  private int rp = 0;
  
  private long lastPress = -1L;
  
  private boolean home = false;
  
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
      double drive = -this.gamepad2.left_stick_y;
      double turn = this.gamepad2.right_stick_x;
      double liftPower = Range.clip(drive + turn, -1.0D, 1.0D);
      this.lf.setPower(v1);
      this.rf.setPower(-v2);
      this.lb.setPower(v3);
      this.rb.setPower(-v4);
      this.lift.setPower(liftPower);
      if (this.gamepad2.b) {
        this.dump.setPower(0.5D);
      } else if (this.gamepad2.a) {
        this.dump.setPower(-0.5D);
      } else {
        this.dump.setPower(0.0D);
      } 
      if (this.gamepad1.a) {
        this.in.setPower(-1.0D);
      } else {
        this.in.setPower(0.0D);
      } 
      if (this.gamepad1.b) {
        this.in.setPower(1.0D);
        continue;
      } 
      this.in.setPower(0.0D);
    } 
  }
  
  public void setupHardware() {
    this.lb = (DcMotor)this.hardwareMap.get(DcMotor.class, "lb");
    this.lf = (DcMotor)this.hardwareMap.get(DcMotor.class, "lf");
    this.rb = (DcMotor)this.hardwareMap.get(DcMotor.class, "rb");
    this.rf = (DcMotor)this.hardwareMap.get(DcMotor.class, "rf");
    this.in = (DcMotor)this.hardwareMap.get(DcMotor.class, "in");
    this.lift = (DcMotor)this.hardwareMap.get(DcMotor.class, "lift");
    this.dump = (CRServo)this.hardwareMap.get(CRServo.class, "dump");
  }
}