/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "OrbitalTeleOp", group = "TeleOp")
public class OrbitalTeleOp extends LinearOpMode {

    private DcMotor lb, lf, rb, rf; //, li, ri, lift;

//    private CRServo lo, ro, lm, rm;

  //  private Servo ctr, ctl, cbr, cbl;

    //private final double intakePower = 0.6;

//    private final double outtakePower = 1;

//    private final int maxPosition = 3000, minPosition = 0;

    private int lp = 0, rp = 0;

    private long lastPress = -1;

    private boolean home = false;

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

            //Intake
/*            if (gamepad1.a) {

                //Let's intake
                li.setPower(-intakePower);
                ri.setPower(intakePower);

            } else if (gamepad1.b) {

                //Outake
                li.setPower(intakePower);
                ri.setPower(-intakePower);


            } else {

                //Stop motion
                li.setPower(0);
                ri.setPower(0);

            }*/

            //Outtake
/*            if (gamepad1.x) {

                lo.setPower(outtakePower);
                ro.setPower(-outtakePower);

            } else if (gamepad1.y) {

                lo.setPower(-outtakePower);
                ro.setPower(outtakePower);

                lm.setPower(1);
                rm.setPower(-1);

            } else {

                lo.setPower(0);
                ro.setPower(0);


            }

            if (gamepad1.x || gamepad1.a) {

                lm.setPower(-1);
                rm.setPower(1);

            } else if (gamepad1.y || gamepad1.b) {

                lm.setPower(1);
                rm.setPower(-1);

            } else {

                lm.setPower(0);
                rm.setPower(0);

            }

            //Grabber Servoz
            if (gamepad2.a) {

                if (home) {
                    ctl.setPosition(0.52);
                    cbl.setPosition(0.52);
                    cbr.setPosition(0.57);
                    ctr.setPosition(0.57);
                    home = false;
                }

            }
            else if (gamepad2.y)
            {

                if (home) {
                    ctl.setPosition(0.47);
                    ctl.setPosition(0.47);
                    ctl.setPosition(0.62);
                    ctl.setPosition(0.62);
                    home = false;
                }
            }
            else {

                if (!home) {
                    ctl.setPosition(0.77);
                    cbl.setPosition(0.77);
                    cbr.setPosition(0.32);
                    ctr.setPosition(0.32);
                    home = true;
                }

            }

            //Lift
            int currentPosition = lift.getCurrentPosition();
            if (-gamepad2.left_stick_y < 0) {
                if (Math.abs(currentPosition - minPosition) >= 10) {
                    lift.setPower(-gamepad2.left_stick_y);
                }
            } else if (-gamepad2.left_stick_y > 0) {
                if (Math.abs(currentPosition - maxPosition) >= 10) {
                    lift.setPower(-gamepad2.left_stick_y);
                }
            } else {
                lift.setPower(0);
            }
            lift.setPower(-gamepad2.left_stick_y);
            telemetry.addData("Pos", lift.getCurrentPosition());
            telemetry.update();
*/
        }

    }

    public void setupHardware() {

        //Initialize motors and other hardware
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        //li = hardwareMap.get(DcMotor.class, "li");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        /*ri = hardwareMap.get(DcMotor.class, "ri");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lo = hardwareMap.get(CRServo.class, "lo");
        ro = hardwareMap.get(CRServo.class, "ro");
        lm = hardwareMap.get(CRServo.class, "lm");
        rm = hardwareMap.get(CRServo.class, "rm");
        ctl = hardwareMap.get(Servo.class, "ctl");
        ctr = hardwareMap.get(Servo.class, "ctr");
        cbl = hardwareMap.get(Servo.class, "cbl");
        cbr = hardwareMap.get(Servo.class, "cbr");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
    }

}
