package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "OpenCVAutov1", group = "Autonomous")
public class OpenCVAutov1 extends LinearOpMode {
  OpenCvCamera webcam;
  
  private DcMotor lb;
  
  private DcMotor lf;
  
  private DcMotor rb;
  
  private DcMotor rf;
  
  SkystoneDeterminationPipeline pipeline;
  
  public void runOpMode() {
    int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
    this.webcam = (OpenCvCamera)OpenCvCameraFactory.getInstance().createWebcam((WebcamName)this.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    this.pipeline = new SkystoneDeterminationPipeline();
    this.webcam.setPipeline(this.pipeline);
    this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
          public void onOpened() {
            OpenCVAutov1.this.webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
          }
        });
    this.telemetry.addLine("Waiting for start");
    this.telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      this.telemetry.addData("Analysis", Integer.valueOf(this.pipeline.getAnalysis()));
      this.telemetry.addData("Position", this.pipeline.position);
      this.telemetry.update();
      sleep(50L);
    } 
  }
  
  public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
    public enum RingPosition {
      FOUR, ONE, NONE;
    }
    
    static final Scalar BLUE = new Scalar(0.0D, 0.0D, 255.0D);
    
    static final Scalar GREEN = new Scalar(0.0D, 255.0D, 0.0D);
    
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(139.0D, 155.0D);
    
    static final int REGION_WIDTH = 40;
    
    static final int REGION_HEIGHT = 40;
    
    final int FOUR_RING_THRESHOLD = 150;
    
    final int ONE_RING_THRESHOLD = 130;
    
    Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    
    Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + 40.0D, REGION1_TOPLEFT_ANCHOR_POINT.y + 40.0D);
    
    Mat region1_Cb;
    
    Mat YCrCb = new Mat();
    
    Mat Cb = new Mat();
    
    int avg1;
    
    private volatile RingPosition position = RingPosition.FOUR;
    
    void inputToCb(Mat input) {
      Imgproc.cvtColor(input, this.YCrCb, 37);
      Core.extractChannel(this.YCrCb, this.Cb, 1);
    }
    
    public void init(Mat firstFrame) {
      inputToCb(firstFrame);
      this.region1_Cb = this.Cb.submat(new Rect(this.region1_pointA, this.region1_pointB));
    }
    
    public Mat processFrame(Mat input) {
      inputToCb(input);
      this.avg1 = (int)(Core.mean(this.region1_Cb)).val[0];
      Imgproc.rectangle(input, this.region1_pointA, this.region1_pointB, BLUE, 2);
      this.position = RingPosition.FOUR;
      if (this.avg1 > 150) {
        this.position = RingPosition.FOUR;
      } else if (this.avg1 > 130) {
        this.position = RingPosition.ONE;
      } else {
        this.position = RingPosition.NONE;
      } 
      Imgproc.rectangle(input, this.region1_pointA, this.region1_pointB, GREEN, -1);
      return input;
    }
    
    public int getAnalysis() {
      return this.avg1;
    }
  }
  
  public enum RingPosition {
    FOUR, ONE, NONE;
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