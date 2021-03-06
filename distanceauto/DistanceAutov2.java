package org.firstinspires.ftc.teamcode.distanceauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DistanceAutov2", group = "Autonomous")
public class DistanceAutov2 extends LinearOpMode {
  private DcMotor lb;

  private DcMotor lf;

  private DcMotor rb;

  private DcMotor rf;

  private DcMotor arm;

  private CRServo grab;

  boolean closed = true;

  public void runOpMode() {
    setupHardware();
    waitForStart();
    setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);
    //right(30, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    //forward(3100, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    //left(100, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    //backward(3075, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    //right(500, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    //forward(3800, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    //right(250, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    //backward(1200, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    armDrop(20);
    while (opModeIsActive()) {
      if (closed)
        grab.setPower(0.2);
      else
        grab.setPower(-0.2);
    }

  }

  public void setupHardware() {
    lb = hardwareMap.get(DcMotor.class, "lb");
    lf = hardwareMap.get(DcMotor.class, "lf");
    rb = hardwareMap.get(DcMotor.class, "rb");
    rf = hardwareMap.get(DcMotor.class, "rf");
    arm = hardwareMap.get(DcMotor.class, "arm");
  }

  public void stopRobot() {
    rf.setPower(0.0D);
    rb.setPower(0.0D);
    lf.setPower(0.0D);
    lb.setPower(0.0D);
    arm.setPower(0.0D);
  }

  public void setRunMode(DcMotor.RunMode runMode) {
    lb.setMode(runMode);
    lf.setMode(runMode);
    rb.setMode(runMode);
    rf.setMode(runMode);
  }

  public void setStopMode(DcMotor.ZeroPowerBehavior zpb) {
    lb.setZeroPowerBehavior(zpb);
    lf.setZeroPowerBehavior(zpb);
    rb.setZeroPowerBehavior(zpb);
    rf.setZeroPowerBehavior(zpb);
    arm.setZeroPowerBehavior(zpb);
  }

  public void forward(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    rf.setTargetPosition(rf.getCurrentPosition() + amt);
    rb.setTargetPosition(rb.getCurrentPosition() + amt);
    lf.setTargetPosition(lf.getCurrentPosition() - amt);
    lb.setTargetPosition(lb.getCurrentPosition() - amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy() && opModeIsActive()) {
      double prop = (amt - rf.getTargetPosition() - rf.getCurrentPosition()) / amt;
      telemetry.addData("P", Double.valueOf(prop));
      telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      rf.setPower(adjPower);
      lf.setPower(adjPower);
      rb.setPower(adjPower);
      lb.setPower(adjPower);
    }
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void backward(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    rf.setTargetPosition(rf.getCurrentPosition() - amt);
    rb.setTargetPosition(rb.getCurrentPosition() - amt);
    lf.setTargetPosition(lf.getCurrentPosition() + amt);
    lb.setTargetPosition(lb.getCurrentPosition() + amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy() && opModeIsActive()) {
      double prop = (amt - rf.getTargetPosition() - rf.getCurrentPosition()) / amt;
      telemetry.addData("P", Double.valueOf(prop));
      telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      rf.setPower(adjPower);
      lf.setPower(adjPower);
      rb.setPower(adjPower);
      lb.setPower(adjPower);
    }
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void right(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    rf.setTargetPosition(rf.getCurrentPosition() - amt);
    rb.setTargetPosition(rb.getCurrentPosition() + amt);
    lf.setTargetPosition(lf.getCurrentPosition() - amt);
    lb.setTargetPosition(lb.getCurrentPosition() + amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy() && opModeIsActive()) {
      double prop = (amt - rf.getTargetPosition() - rf.getCurrentPosition()) / amt;
      telemetry.addData("P", Double.valueOf(prop));
      telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      rf.setPower(adjPower);
      lf.setPower(adjPower);
      rb.setPower(adjPower);
      lb.setPower(adjPower);
    }
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void left(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    rf.setTargetPosition(rf.getCurrentPosition() + amt);
    rb.setTargetPosition(rb.getCurrentPosition() - amt);
    lf.setTargetPosition(lf.getCurrentPosition() + amt);
    lb.setTargetPosition(lb.getCurrentPosition() - amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy() && opModeIsActive()) {
      double prop = (amt - rf.getTargetPosition() - rf.getCurrentPosition()) / amt;
      telemetry.addData("P", Double.valueOf(prop));
      telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      rf.setPower(adjPower);
      lf.setPower(adjPower);
      rb.setPower(adjPower);
      lb.setPower(adjPower);
    }
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void armDrop(int amt) {

    arm.setTargetPosition(amt);
    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (arm.isBusy() && opModeIsActive())
      arm.setPower(0.75);
    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  }

  public double getPower(double amtDone, double edge1, double edge2, double max, double min, double p1, double p2) {
    if (amtDone >= edge1 && amtDone <= 1.0D - edge2)
      return max;
    if (amtDone > 1.0D - edge2) {
      amtDone = 1.0D - amtDone;
      double d = amtDone / edge2;
      return min + evaluateNormal(1.0D, p2, d, max - min);
    }
    double amtNormDone = amtDone / edge1;
    return min + evaluateNormal(1.0D, p1, amtNormDone, max - min);
  }

  public double evaluateNormal(double m, double s, double x, double max) {
    double power = -Math.pow(x - m, 2.0D) / 2.0D * Math.pow(s, 2.0D);
    return max * Math.pow(Math.E, power);
  }
}