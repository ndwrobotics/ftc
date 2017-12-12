/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder Motor Calibration", group="Calibration")  // @Autonomous(...) is the other common choice
//@Disabled
public class CalibrationMotorEncoder extends LinearOpMode
{


    @Override
    public void runOpMode() {


        DcMotor encoderMotor = hardwareMap.dcMotor.get("encoder motor");


        boolean x_then = false;
        boolean x_now;
        boolean y_then = false;
        boolean y_now;
        boolean a_then = false;
        boolean a_now;
        boolean b_then = false;
        boolean b_now;


        int motorPosition = 0;
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElapsedTime r = new ElapsedTime();
        s(r, 1000);
        encoderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        s(r, 1000);

        waitForStart();

        while (opModeIsActive()) {
            a_now = gamepad1.a;
            b_now = gamepad1.b;
            x_now = gamepad1.x;
            y_now = gamepad1.y;
            if (a_now && !a_then) {
                encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                s(r, 1000);
                encoderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                s(r, 1000);
                motorPosition = 0;
            }
            if (b_now && !b_then) {
                motorPosition += 10;
            }

            if (y_now && !y_then) {
                motorPosition -= 10;
            }
            if (x_now && !x_then) {
                motorPosition += 1;
            }

            encoderMotor.setTargetPosition(motorPosition);
            encoderMotor.setPower(0.2);
            telemetry.addData("motor position: ", motorPosition);
            telemetry.update();
            a_then = a_now;
            b_then = b_now;
            x_then = x_now;
            y_then = y_now;

        }
    }

    public void s(ElapsedTime runtime, double milliseconds){
        double seconds = runtime.seconds();
        while(opModeIsActive() && runtime.seconds() < seconds){}
    }

}
// one second per foot at 0.2 power
// -40 thru 420