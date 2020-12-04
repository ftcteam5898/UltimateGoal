package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DistanceAutov2", group = "Autonomous")
public class DistanceAutov2 extends LinearOpMode {
  private DcMotor lb;
  
  private DcMotor lf;
  
  private DcMotor rb;
  
  private DcMotor rf;
  
  public void runOpMode() {
    setupHardware();
    waitForStart();
    setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);
    right(30, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    forward(3100, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    left(100, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    backward(3075, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    right(500, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    forward(3800, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    right(250, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    backward(1200, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    while (opModeIsActive());
  }
  
  public void setupHardware() {
    this.lb = (DcMotor)this.hardwareMap.get(DcMotor.class, "lb");
    this.lf = (DcMotor)this.hardwareMap.get(DcMotor.class, "lf");
    this.rb = (DcMotor)this.hardwareMap.get(DcMotor.class, "rb");
    this.rf = (DcMotor)this.hardwareMap.get(DcMotor.class, "rf");
  }
  
  public void stopRobot() {
    this.rf.setPower(0.0D);
    this.rb.setPower(0.0D);
    this.lf.setPower(0.0D);
    this.lb.setPower(0.0D);
  }
  
  public void setRunMode(DcMotor.RunMode runMode) {
    this.lb.setMode(runMode);
    this.lf.setMode(runMode);
    this.rb.setMode(runMode);
    this.rf.setMode(runMode);
  }
  
  public void setStopMode(DcMotor.ZeroPowerBehavior zpb) {
    this.lb.setZeroPowerBehavior(zpb);
    this.lf.setZeroPowerBehavior(zpb);
    this.rb.setZeroPowerBehavior(zpb);
    this.rf.setZeroPowerBehavior(zpb);
  }
  
  public void forward(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    this.rf.setTargetPosition(this.rf.getCurrentPosition() + amt);
    this.rb.setTargetPosition(this.rb.getCurrentPosition() + amt);
    this.lf.setTargetPosition(this.lf.getCurrentPosition() - amt);
    this.lb.setTargetPosition(this.lb.getCurrentPosition() - amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (this.rf.isBusy() && this.lf.isBusy() && this.rb.isBusy() && this.lb.isBusy() && opModeIsActive()) {
      double prop = (amt - this.rf.getTargetPosition() - this.rf.getCurrentPosition()) / amt;
      this.telemetry.addData("P", Double.valueOf(prop));
      this.telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      this.rf.setPower(adjPower);
      this.lf.setPower(adjPower);
      this.rb.setPower(adjPower);
      this.lb.setPower(adjPower);
    } 
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
  public void backward(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    this.rf.setTargetPosition(this.rf.getCurrentPosition() - amt);
    this.rb.setTargetPosition(this.rb.getCurrentPosition() - amt);
    this.lf.setTargetPosition(this.lf.getCurrentPosition() + amt);
    this.lb.setTargetPosition(this.lb.getCurrentPosition() + amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (this.rf.isBusy() && this.lf.isBusy() && this.rb.isBusy() && this.lb.isBusy() && opModeIsActive()) {
      double prop = (amt - this.rf.getTargetPosition() - this.rf.getCurrentPosition()) / amt;
      this.telemetry.addData("P", Double.valueOf(prop));
      this.telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      this.rf.setPower(adjPower);
      this.lf.setPower(adjPower);
      this.rb.setPower(adjPower);
      this.lb.setPower(adjPower);
    } 
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
  public void right(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    this.rf.setTargetPosition(this.rf.getCurrentPosition() - amt);
    this.rb.setTargetPosition(this.rb.getCurrentPosition() + amt);
    this.lf.setTargetPosition(this.lf.getCurrentPosition() - amt);
    this.lb.setTargetPosition(this.lb.getCurrentPosition() + amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (this.rf.isBusy() && this.lf.isBusy() && this.rb.isBusy() && this.lb.isBusy() && opModeIsActive()) {
      double prop = (amt - this.rf.getTargetPosition() - this.rf.getCurrentPosition()) / amt;
      this.telemetry.addData("P", Double.valueOf(prop));
      this.telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      this.rf.setPower(adjPower);
      this.lf.setPower(adjPower);
      this.rb.setPower(adjPower);
      this.lb.setPower(adjPower);
    } 
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
  public void left(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    this.rf.setTargetPosition(this.rf.getCurrentPosition() + amt);
    this.rb.setTargetPosition(this.rb.getCurrentPosition() - amt);
    this.lf.setTargetPosition(this.lf.getCurrentPosition() + amt);
    this.lb.setTargetPosition(this.lb.getCurrentPosition() - amt);
    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (this.rf.isBusy() && this.lf.isBusy() && this.rb.isBusy() && this.lb.isBusy() && opModeIsActive()) {
      double prop = (amt - this.rf.getTargetPosition() - this.rf.getCurrentPosition()) / amt;
      this.telemetry.addData("P", Double.valueOf(prop));
      this.telemetry.update();
      double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);
      this.rf.setPower(adjPower);
      this.lf.setPower(adjPower);
      this.rb.setPower(adjPower);
      this.lb.setPower(adjPower);
    } 
    stopRobot();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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