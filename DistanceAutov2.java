package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DistanceAutov2", group = "Autonomous")

public class DistanceAutov2 extends LinearOpMode {

    private DcMotor lb, lf, rb, rf;
    
    @Override
    public void runOpMode() {

        setupHardware();
        waitForStart();

        //All right, let's see what we can do!
        setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);
        forward(2800, 0.1, 0.5, 0.65, 0.15, 0.333, 0.333);
        left(100, 0.1, 0.5, 0.65, 0.15, 0.333,0.333);
        backward(2775, 0.1, 0.5, 0.65, 0.15, 0.333, 0.333);
        right(550, 0.1, 0.5, 0.65, 0.15, 0.333, 0.333);
        forward(3800, 0.1, 0.5, 0.65, 0.15, 0.333, 0.333);
        backward(1200, 0.1, 0.5, 0.65, 0.15, 0.333, 0.333);

        while (opModeIsActive()) {}
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
