package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "OrbitalTeleOpv2", group = "TeleOp")
public class OrbitalTeleOpv2 extends LinearOpMode {

    private DcMotor lb, lf, rb, rf, in, arm;
    private CRServo grab;
    private double grabPos = 0; 


    @Override
    public void runOpMode() {
        
        
        setupHardware();
        waitForStart();

        //Enter main TeleOp loop
        while (opModeIsActive()) {
             
            //Calculate mecanum powers with turning
            double r = Math.hypot(gamepad1.left_stick_x - gamepad1.right_stick_x, gamepad1.left_stick_y - gamepad1.right_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y - gamepad1.right_stick_y, -gamepad1.left_stick_x + gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_trigger - gamepad1.left_trigger;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;
            

            //Set motor powers
            lf.setPower(v1);
            rf.setPower(-v2);
            lb.setPower(v3);
            rb.setPower(-v4);
   
           
            
            if (gamepad1.a)
                in.setPower(-1.0);
            else
                in.setPower(0.0);
                
            if (gamepad1.b)
                in.setPower(1.0);
            else
                in.setPower(0.0);
            
            if (gamepad2.b)
                arm.setPower(1.0);
            else
                arm.setPower(0.0);
            
            if (gamepad2.a)
                arm.setPower(-1.0);
            else
                arm.setPower(0.0);
            
            if (gamepad2.x)
                grab.setPower(0.2);
            else if (gamepad2.y)
                grab.setPower(-0.2);
            else
		grab.setPower(0.0);
            
        }

    }

    public void setupHardware() {

        //Initialize motors and other hardware
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        in = hardwareMap.get(DcMotor.class, "in");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grab = hardwareMap.get(CRServo.class, "grab");

    }

}
