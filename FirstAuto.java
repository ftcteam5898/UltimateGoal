package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "FirstAuto1.0", group = "Autonomous")

public class FirstAuto extends LinearOpMode {

    // todo: write your code here
    private DcMotor lb, lf, rb, rf;
    
    private BNO055IMU imu2;
    private Orientation angles2;
    
    private String position;
    
    private double headingO;
    
    @Override
    public void runOpMode() {
        
        setupHardware();
        setStopMode(DcMotor.ZeroPowerBehavior.BRAKE);
        heading0 = getHeading();
        // heading0 += 180;
        while (!isStarted())
    }
}
