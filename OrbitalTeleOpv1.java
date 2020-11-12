package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "OrbitalTeleOpv1", group = "TeleOp")
public class OrbitalTeleOpv1 extends LinearOpMode {

    private DcMotor lb, lf, rb, rf, in, lift;
    private CRServo dump;
     
    private int lp = 0, rp = 0;

    private long lastPress = -1;

    private boolean home = false;

    @Override
    public void runOpMode() {

        setupHardware();
        waitForStart();

        //Enter main TeleOp loop
        while (opModeIsActive()) {
            double liftPower;
            //lift.setPower(liftPower);
             
            //Calculate mecanum powers with turning
            double r = Math.hypot(gamepad1.left_stick_x - gamepad1.right_stick_x, gamepad1.left_stick_y - gamepad1.right_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y - gamepad1.right_stick_y, -gamepad1.left_stick_x + gamepad1.right_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_trigger - gamepad1.left_trigger;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;
//Calculate gamepad2 Dump arm

           // lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //lift.setMode
            double drive = -gamepad2.left_stick_y;
            double turn  =  gamepad2.right_stick_x;
            liftPower    = Range.clip(drive + turn, -1.0, 1.0) ;

            //Set motor powers
            lf.setPower(v1);
            rf.setPower(-v2);
            lb.setPower(v3);
            rb.setPower(-v4);
            lift.setPower(liftPower);
            if(gamepad2.b)
           {
               dump.setPower(0.5);
           }
           else if (gamepad2.a)
           {
               dump.setPower(-0.5);
           }
           else
           {
               dump.setPower(0);
           }
           
            
            if (gamepad1.a)
                in.setPower(-1.0);
            else
                in.setPower(0.0);
                
            if (gamepad1.b)
                in.setPower(1.0);
            else
                in.setPower(0.0);
                
            
            

        }

    }

    public void setupHardware() {

        //Initialize motors and other hardware
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        in = hardwareMap.get(DcMotor.class, "in");
        lift = hardwareMap.get(DcMotor.class,"lift");
        dump = hardwareMap.get(CRServo.class,"dump");

    }

}
