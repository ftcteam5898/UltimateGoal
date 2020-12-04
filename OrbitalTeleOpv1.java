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
      double r = Math.hypot((gamepad1.left_stick_x - gamepad1.right_stick_x), (gamepad1.left_stick_y - gamepad1.right_stick_y));
      double robotAngle = Math.atan2((gamepad1.left_stick_y - gamepad1.right_stick_y), (-gamepad1.left_stick_x + gamepad1.right_stick_x)) - 0.7853981633974483D;
      double rightX = (gamepad1.right_trigger - gamepad1.left_trigger);
      double v1 = r * Math.cos(robotAngle) - rightX;
      double v2 = r * Math.sin(robotAngle) + rightX;
      double v3 = r * Math.sin(robotAngle) - rightX;
      double v4 = r * Math.cos(robotAngle) + rightX;
      double drive = -gamepad2.left_stick_y;
      double turn = gamepad2.right_stick_x;
      double liftPower = Range.clip(drive + turn, -1.0D, 1.0D);
      lf.setPower(v1);
      rf.setPower(-v2);
      lb.setPower(v3);
      rb.setPower(-v4);
      lift.setPower(liftPower);
      if (gamepad2.b) {
        dump.setPower(0.5D);
      } else if (gamepad2.a) {
        dump.setPower(-0.5D);
      } else {
        dump.setPower(0.0D);
      } 
      if (gamepad1.a) {
        in.setPower(-1.0D);
      } else {
        in.setPower(0.0D);
      } 
      if (gamepad1.b) {
        in.setPower(1.0D);
      } 
      in.setPower(0.0D);
    } 
  }
  
  public void setupHardware() {
    lb = hardwareMap.get(DcMotor.class, "lb");
    lf = hardwareMap.get(DcMotor.class, "lf");
    rb = hardwareMap.get(DcMotor.class, "rb");
    rf = hardwareMap.get(DcMotor.class, "rf");
    in = hardwareMap.get(DcMotor.class, "in");
    lift = hardwareMap.get(DcMotor.class, "lift");
    dump = hardwareMap.get(CRServo.class, "dump");
  }
}