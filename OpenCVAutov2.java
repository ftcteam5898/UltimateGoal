package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

@Autonomous(name = "OpenCVAutov2", group = "Autonomous")
public class OpenCVAutov2 extends LinearOpMode {
  OpenCvCamera webcam;
  
  private DcMotor lb;
  
  private DcMotor lf;
  
  private DcMotor rb;
  
  private DcMotor rf;
  
  private DcMotor arm;
  
  private CRServo grab;
  
  SkystoneDeterminationPipeline pipeline;
  
  public void runOpMode() {
    int cameraMonitorViewId = this.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hardwareMap.appContext.getPackageName());
    this.webcam = (OpenCvCamera)OpenCvCameraFactory.getInstance().createWebcam((WebcamName)this.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    this.pipeline = new SkystoneDeterminationPipeline();
    this.webcam.setPipeline(this.pipeline);
    this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
          public void onOpened() {
            OpenCVAutov2.this.webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
          }
        });
    this.telemetry.addLine("Waiting for start");
    this.telemetry.update();
    setupHardware();
    waitForStart();
    setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);
    int[] positionVal = new int[50];
    for (int i = 0; i < 20; i++) {
      this.telemetry.addData("Analysis", Integer.valueOf(this.pipeline.getAnalysis()));
      this.telemetry.addData("Position", this.pipeline.position);
      if (this.pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
        positionVal[i] = 4;
      } else if (this.pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
        positionVal[i] = 1;
      } else {
        positionVal[i] = 0;
      } 
      this.telemetry.update();
      sleep(50L);
    } 
    int pos = mode(positionVal);
    if (pos == 0) {
      this.telemetry.addData("x", "0");
      this.telemetry.update();
      left(300, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(2500, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      right(1000, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-2500, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      left(550, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(2400, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-100, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      left(500, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(100, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    } else if (pos == 1) {
      this.telemetry.addData("x", "1");
      this.telemetry.update();
      left(300, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(3300, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      right(250, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-200, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      left(300, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-2800, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      right(850, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(2900, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      left(450, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-200, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    } else {
      this.telemetry.addData("x", "4");
      this.telemetry.update();
      right(30, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(4000, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      right(550, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-200, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      left(550, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-2675, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      right(550, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(3800, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      right(250, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
      forward(-1200, 0.1D, 0.5D, 0.65D, 0.15D, 0.333D, 0.333D);
    } 
    while (opModeIsActive());
  }
  
  public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
    public enum RingPosition {
      FOUR, ONE, NONE;
    }
    
    static final Scalar BLUE = new Scalar(0.0D, 0.0D, 255.0D);
    
    static final Scalar GREEN = new Scalar(0.0D, 255.0D, 0.0D);
    
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(145.0D, 110.0D);
    
    static final int REGION_WIDTH = 40;
    
    static final int REGION_HEIGHT = 40;
    
    final int FOUR_RING_THRESHOLD = 140;
    
    final int ONE_RING_THRESHOLD = 128;
    
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
      if (this.avg1 > 140) {
        this.position = RingPosition.FOUR;
      } else if (this.avg1 > 128) {
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
    this.arm = (DcMotor)this.hardwareMap.get(DcMotor.class, "arm");
    this.grab = (CRServo)this.hardwareMap.get(CRServo.class, "grab");
  }
  
  public void stopRobot() {
    this.rf.setPower(0.0D);
    this.rb.setPower(0.0D);
    this.lf.setPower(0.0D);
    this.lb.setPower(0.0D);
    this.arm.setPower(0.0D);
    this.grab.setPower(0.0D);
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
    this.arm.setZeroPowerBehavior(zpb);
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
  
  public void spin(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
    this.rf.setTargetPosition(this.rf.getCurrentPosition() + amt);
    this.rb.setTargetPosition(this.rb.getCurrentPosition() + amt);
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
  
  public void ringDrop() {
    this.arm.setPower(1.0D);
    runMs(100L);
    this.arm.setPower(-0.3D);
    this.grab.setPower(-0.1D);
    runMs(50L);
    this.grab.setPower(0.1D);
    this.arm.setPower(-1.0D);
    runMs(100L);
    this.grab.setPower(0.0D);
    this.arm.setPower(0.0D);
  }
  
  public void runMs(long ms) {
    long end = System.currentTimeMillis();
    while (System.currentTimeMillis() - end < ms && opModeIsActive());
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
  
  public static int mode(int[] a) {
    int maxValue = 0, maxCount = 0;
    for (int i = 0; i < 20; i++) {
      int count = 0;
      for (int j = 0; j < 20; j++) {
        if (a[j] == a[i])
          count++; 
      } 
      if (count > maxCount) {
        maxCount = count;
        maxValue = a[i];
      } 
    } 
    return maxValue;
  }
}
