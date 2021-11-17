package org.firstinspires.ftc.teamcode.opencv;

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

public class OpenCVAutov1 extends LinearOpMode
{
  OpenCvCamera webcam;
  private DcMotor lb, lf, rb, rf;
  SkystoneDeterminationPipeline pipeline;

  @Override
  public void runOpMode()
  {

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    pipeline = new DuckPipeline();
    webcam.setPipeline(pipeline);

    // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
    // out when the RC activity is in portrait. We do our actual image processing assuming
    // landscape orientation, though.

    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
      }
    });

    telemetry.addLine("Waiting for start");
    telemetry.update();

    waitForStart();

    while (opModeIsActive())
    {
      telemetry.addData("Analysis 1", pipeline.getAnalysis1());
      telemetry.addData("Analysis 2", pipeline.getAnalysis2());
      telemetry.addData("Position", pipeline.position);
      telemetry.update();



      // Don't burn CPU cycles busy-looping in this sample
      sleep(50);
    }
  }

  public static class DuckPipeline extends OpenCvPipeline
  {
    /*
     * An enum to define the ring position
     */
    public enum DuckPosition
    {
      LEFT,
      MIDDLE,
      RIGHT
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(130.0D, 140.0D);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(130.0D, 140.0D);
    // 139 155

    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 40;

    final int LEFT_THRESHOLD = 144;
    final int MIDDLE_THRESHOLD = 130;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGIO2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat region2_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1;
    int avg2;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile DuckPosition position = DuckPosition.LEFT;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
      Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame)
    {
      inputToCb(firstFrame);

      region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
      region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
      inputToCb(input);

      avg1 = (int) Core.mean(region1_Cb).val[0];
      avg2 = (int) Core.mean(region2_Cb).val[0];

      Imgproc.rectangle(
              input, // Buffer to draw on
              region1_pointA, // First point which defines the rectangle
              region1_pointB, // Second point which defines the rectangle
              BLUE, // The color the rectangle is drawn in
              2); // Thickness of the rectangle lines
      
      Imgproc.rectangle(
              input, // Buffer to draw on
              region_pointA, // First point which defines the rectangle
              region2_pointB, // Second point which defines the rectangle
              BLUE, // The color the rectangle is drawn in
              2); // Thickness of the rectangle lines

      position = DuckPosition.LEFT; // Record our analysis
      if(avg1 > LEFT_THRESHOLD){
        position = DuckPosition.LEFT;
      }else if (avg1 > MIDDLE_THRESHOLD){
        position = DuckPosition.MIDDLE;
      }else{
        position = DuckPosition.RIGHT;
      }

      Imgproc.rectangle(
              input, // Buffer to draw on
              region1_pointA, // First point which defines the rectangle
              region1_pointB, // Second point which defines the rectangle
              GREEN, // The color the rectangle is drawn in
              -1); // Negative thickness means solid fill
      
      Imgproc.rectangle(
              input,
              region2_pointA,
              region2_pointB,
              GREEN,
              -1);

      return input;
    }

    public int getAnalysis1()
    {
      return avg1;
    }

    public int getAnalysis2()
    {
      return avg2;
    }


  }
  public void setupHardware() {

    //Initialize hardware
    lb = hardwareMap.get(DcMotor.class, "lb");
    lf = hardwareMap.get(DcMotor.class, "lf");
    rb = hardwareMap.get(DcMotor.class, "rb");
    rf = hardwareMap.get(DcMotor.class, "rf");

  }

  public void stopRobot() {

    //Stop all motors
    rf.setPower(0);
    rb.setPower(0);
    lf.setPower(0);
    lb.setPower(0);

  }

  public void setRunMode(DcMotor.RunMode runMode) {

    //Change all motor run modes to runMode variable
    lb.setMode(runMode);
    lf.setMode(runMode);
    rb.setMode(runMode);
    rf.setMode(runMode);

  }

  public void setStopMode(DcMotor.ZeroPowerBehavior zpb) {

    //Change all motors to a ZeroPowerBehavior
    lb.setZeroPowerBehavior(zpb);
    lf.setZeroPowerBehavior(zpb);
    rb.setZeroPowerBehavior(zpb);
    rf.setZeroPowerBehavior(zpb);

  }

  public void forward(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {

    rf.setTargetPosition(rf.getCurrentPosition() + amt);
    rb.setTargetPosition(rb.getCurrentPosition() + amt);
    lf.setTargetPosition(lf.getCurrentPosition() - amt);
    lb.setTargetPosition(lb.getCurrentPosition() - amt);

    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    while ((rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy()) && opModeIsActive()) {

      double prop = (amt - (rf.getTargetPosition() - rf.getCurrentPosition())) / (double) amt;
      telemetry.addData("P", prop);
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

    while ((rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy()) && opModeIsActive()) {

      double prop = (amt - (rf.getTargetPosition() - rf.getCurrentPosition())) / (double) amt;
      telemetry.addData("P", prop);
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

    while ((rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy()) && opModeIsActive()) {

      double prop = (amt - (rf.getTargetPosition() - rf.getCurrentPosition())) / (double) amt;
      telemetry.addData("P", prop);
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

    while ((rf.isBusy() && lf.isBusy() && rb.isBusy() && lb.isBusy()) && opModeIsActive()) {

      double prop = (amt - (rf.getTargetPosition() - rf.getCurrentPosition())) / (double) amt;
      telemetry.addData("P", prop);
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

  public double getPower(double amtDone, double edge1, double edge2, double max, double min, double p1, double p2) {

    if (amtDone >= edge1 && amtDone <= 1 - edge2) {

      return max;

    } else {

      if (amtDone > 1 - edge2) {

        amtDone = 1 - amtDone;
        double amtNormDone = amtDone / edge2;
        return min + evaluateNormal(1, p2, amtNormDone, max - min);

      } else {

        double amtNormDone = amtDone / edge1;
        return min + evaluateNormal(1, p1, amtNormDone, max - min);

      }

    }

  }

  public double evaluateNormal(double m, double s, double x, double max) {

    double power = -Math.pow(x - m, 2) / (2 * Math.pow(s, 2));
    return max * Math.pow(Math.E, power);

  }
}