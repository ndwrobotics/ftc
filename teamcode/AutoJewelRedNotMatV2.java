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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="2V5 Jewel+Glyph -Red Not Mat", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class AutoJewelRedNotMatV2 extends LinearOpMode
{


    //--1-begin
    DcMotor wheelR;
    DcMotor wheelL;
    //--1-end
    DcMotor wheelR2;
    DcMotor wheelL2;

    //--2-begin
    Servo leftServo;
    Servo rightServo;
    //--2-end

    DcMotor jewelDisplacer;

    DcMotor lift;

    NormalizedColorSensor colorSensor;
    DcMotor someMotors[];
    DcMotor moreMotors[];


    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        //--1-begin
        wheelR = hardwareMap.dcMotor.get("motorR");
        wheelL = hardwareMap.dcMotor.get("motorL");
        //--1-begin
        wheelR2 = hardwareMap.dcMotor.get("motorR2");
        wheelL2 = hardwareMap.dcMotor.get("motorL2");

        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        wheelL2.setDirection(DcMotorSimple.Direction.REVERSE);


        leftServo = hardwareMap.servo.get("left");
        rightServo = hardwareMap.servo.get("right");

        lift = hardwareMap.dcMotor.get("lift");

        jewelDisplacer = hardwareMap.dcMotor.get("jeweler");


        someMotors = new DcMotor[2];
        moreMotors = new DcMotor[4];

        someMotors[0] = wheelL;
        someMotors[1] = wheelR;

        moreMotors[0] = wheelL;
        moreMotors[1] = wheelR;
        moreMotors[2] = wheelL2;
        moreMotors[3] = wheelR2;


        double UP_SPEED = -0.5; //tweak this
        double DOWN_SPEED = 0.8;


        double LEFT_CENTER = 0.52; //tweak this
        double RIGHT_CENTER = 0.38; //tweak this
        double LEFT_IN = 1.0; //tweak this
        double RIGHT_IN = 0.0; //tweak this
        double LEFT_SLIGHT_OUT = 0.4;
        double RIGHT_SLIGHT_OUT = 0.51;
        double LEFT_OUT = 0.32; //tweak this
        double RIGHT_OUT = 0.6; //tweak this //done
        //--2-end

        int JEWEL_OUT = -580; //tweak this
        int JEWEL_IN = 0; //tweak this //done

        double COLOR_THRESHOLD = 0.71;

        boolean RED_ALLIANCE = true;


        telemetry.addLine("Code is running.");
        telemetry.update();


        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        ElapsedTime r = new ElapsedTime();

        for (DcMotor motor : moreMotors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        jewelDisplacer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jewelDisplacer.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftServo.setPosition(LEFT_CENTER);
        rightServo.setPosition(RIGHT_CENTER);

        waitForStart();


        jewelDisplacer.setTargetPosition(JEWEL_OUT);
        jewelDisplacer.setPower(0.25);
        if(opModeIsActive()){
            lift.setPower(-DOWN_SPEED);
        }
        s(800, r);
        lift.setPower(0);

        displaceJewel(COLOR_THRESHOLD, RED_ALLIANCE);

        jewelDisplacer.setTargetPosition(JEWEL_IN);
        square_up(r, RED_ALLIANCE);
        encoderDrive(6);
        encoderTurn(20);
        timeDrive(600, true, r);
        if(opModeIsActive()){
            lift.setPower(DOWN_SPEED);
        }
        s(800, r);
        lift.setPower(0);
        leftServo.setPosition(LEFT_SLIGHT_OUT);
        rightServo.setPosition(RIGHT_SLIGHT_OUT);
        s(250, r);
        encoderDrive(-8);
        encoderTurn(80);
        encoderDrive(6);
    }

    public void displaceJewel(double color_threshold, int count, boolean redAlliance) {
        if (count < 4 && opModeIsActive()) {
            NormalizedRGBA colors;


            double red = 0;
            double blue = 0;
            float max;
            ElapsedTime r = new ElapsedTime();
            for (int i = 0; i < 10; i++) {
                colors = colorSensor.getNormalizedColors();
                max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red /= max;
                colors.green /= max;
                colors.blue /= max;
                red += colors.red;
                blue += colors.blue;
                telemetry.addLine("normalized color:  ")
                        .addData("a", colors.alpha)
                        .addData("r", colors.red)
                        .addData("g", colors.green)
                        .addData("b", colors.blue);
                telemetry.update();
                s(50, r);
            }
            telemetry.addData("blue: ", blue);
            telemetry.addData("red: ", red);
            telemetry.update();

            double power = 0.15;

            if (opModeIsActive()){
                if (red > 10 * color_threshold) {
                    if (redAlliance){
                        encoderPowerTurn(30, power);
                        encoderPowerTurn(-30, power);
                    } else {
                        encoderPowerTurn(-30, power);
                        encoderPowerTurn(30, power);
                    }
                } else if (blue > 10 * color_threshold) {
                    if(redAlliance){
                        encoderPowerTurn(-30, power);
                        encoderPowerTurn(30, power);
                    } else {
                        encoderPowerTurn(30, power);
                        encoderPowerTurn(-30, power);
                    }

                } else {
                    if (count == 0){
                        encoderPowerDrive(0.5, 0.15);
                    } else if (count == 1){
                        encoderPowerDrive(-1, 0.15);
                    } else if (count == 2){
                        encoderPowerDrive(-0.5, 0.15);
                    } else if (count == 3){
                        encoderPowerDrive(-0.5, 0.15);
                    }
                    displaceJewel(color_threshold, count + 1, redAlliance);
                }
            }
        }
    }

    public void displaceJewel(double color_threshold, boolean redAlliance){
        displaceJewel(color_threshold, 0, redAlliance);
    }


    public void s(double time, ElapsedTime r){
        double t = r.milliseconds();
        while (opModeIsActive() && r.milliseconds() <= t + time){}
    }

    public void encoderPowerDrive(double inches, double power){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        if (opModeIsActive()) {
            for (DcMotor motor : someMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : someMotors){
                motor.setPower(power);
            }
            while (opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy())){
                telemetry.addData("Target position: ", inches);
                telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                telemetry.update();
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }

    public void encoderDrive(double inches){
        encoderPowerDrive(inches, 0.25);
    }
    public void encoder4WDrive(double inches){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);

        if (opModeIsActive()) {
            for (DcMotor motor : moreMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0.25);
            }
            while (opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy() || wheelL2.isBusy() || wheelR2.isBusy())){
                telemetry.addData("Target position: ", inches);
                telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                telemetry.update();
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }

    public void encoderPowerTurn(double degrees, double power){
        // goes counterclockwise

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 280/pi counts per inch
        // turn diameter is 13.75 inches, so a 360 degree turn is 13.75pi inches
        // so one degree is 13.75pi/360 = 55pi/1440 = 11pi/288
        // 280/pi counts per inch times 11pi/288 inches per degree is 11*280/288 counts per degree
        // 11*280/288 = 11*35/36 = 385/36
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 17.0/16.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        if (opModeIsActive()) {
            wheelL.setTargetPosition(wheelL.getCurrentPosition() - counts);
            wheelR.setTargetPosition(wheelR.getCurrentPosition() + counts);
            // should go counterclockwise
            for (DcMotor motor : someMotors){
                motor.setPower(power);
            }
            while (opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy())){
                telemetry.addData("Target position: ", counts);
                telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                telemetry.update();
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());
        }
    }

    public void encoderTurn(double degrees){
        encoderPowerTurn(degrees, 0.25);
    }

    public void timePowerDrive(double milliseconds, boolean forwards, double power, ElapsedTime r){
        for (DcMotor motor : moreMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotor motor : moreMotors){
            if (forwards){
                motor.setPower(power);
            } else {
                motor.setPower(-power);
            }
        }
        s(milliseconds, r);
        for (DcMotor motor : moreMotors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        s(250, r);
    }

    public void timeDrive(double milliseconds, boolean forwards, ElapsedTime r){
        timePowerDrive(milliseconds, forwards, 0.25, r);
    }

    public void square_up(ElapsedTime r, boolean forwards){

        if (forwards){
            timeDrive(1000, true, r);
            timePowerDrive(1000, false, 0.15, r);
        } else {
            timeDrive(1000, false, r);
            timePowerDrive(1000, true, 0.15, r);
        }
        s(250, r);
    }

}
// one second per foot at 0.2 power
