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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="2V4 Vuforia -Red Mat", group="Autonomous")  // @Autonomous(...) is the other common choice
//@Disabled
public class Auto2V4RedMat extends LinearOpMode
{


    //--1-begin
    DcMotor wheelR;
    DcMotor wheelL;
    //--1-end

    //--5V2-begin
    DcMotor wheelR2;
    DcMotor wheelL2;
    //--5V2-end

    //--2-begin
    Servo leftServo;
    Servo rightServo;
    //--2-end

    //--4-begin
    DcMotor lift;
    //--4-end

    //--5-begin
    TouchSensor touchSensor;
    //--5-end

    DcMotor someMotors[] = new DcMotor[2];
    DcMotor moreMotors[] = new DcMotor[4];


    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        //--1-begin
        wheelR = hardwareMap.dcMotor.get("motorR");
        wheelL = hardwareMap.dcMotor.get("motorL");

        //--1-begin
        wheelR2 = hardwareMap.dcMotor.get("motorR2");
        wheelL2 = hardwareMap.dcMotor.get("motorL2");

        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        wheelL2.setDirection(DcMotorSimple.Direction.REVERSE);
        //--2-begin

        leftServo = hardwareMap.servo.get("left");
        rightServo = hardwareMap.servo.get("right");

        //--6-begin


        someMotors[0] = wheelL;
        someMotors[1] = wheelR;

        moreMotors[0] = wheelL;
        moreMotors[1] = wheelR;
        moreMotors[2] = wheelL2;
        moreMotors[3] = wheelR2;





        int motorPosition = 0;
        //--6-end

        double LEFT_CENTER = 0.52; //tweak this
        double RIGHT_CENTER = 0.38; //tweak this
        double LEFT_IN = 1.0; //tweak this
        double RIGHT_IN = 0.0; //tweak this
        double LEFT_SLIGHT_OUT = 0.42;
        double RIGHT_SLIGHT_OUT = 0.49;
        double LEFT_OUT = 0.32; //tweak this
        double RIGHT_OUT = 0.6; //tweak this //done
        //--2-end

        //--4-begin
        telemetry.addLine("Getting lift motor...");
        telemetry.update();
        lift = hardwareMap.dcMotor.get("lift");

        double UP_SPEED = -0.5; //tweak this
        double DOWN_SPEED = 0.8;
        //--4-end


        //--5-begin
        touchSensor = hardwareMap.touchSensor.get("touchUp");

        wheelL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheelR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "Aczwc7j/////AAAAGSh6oYrgWE/wvIHsaTUy6ppNpIawgvZu5xiQAUK+xtcW8Zw8P9GhxqBOsB2RKBAP6cOUkwCiyhCaYHKfrhA7ORLNRka9TTTm36bBHMY/WhR02K5Z5Qa2TaqRNjtpZ5rZ2Q8Ee+vnuiVasqj3uAMCB0ceFPYemYJ8snub+w/8gKVwy09n+ZWJ/5yymbJ9lGIQAwlc8Wo0HogRSK6Yk0Z0CiX/UFKhB+wK+I7Vzcwv0pZVOXAhf7fuDJOUr+TcuCBptm6AlFai2evVFuKT1seKaTsX+Sa2jvP3F0O1gbfLYK1dWjIh5/mqEp6yWgk2D+fAhGGWVjeUkN0ftstV6F1U06+f9P52sOd6w0Fuagpm27TP";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        telemetry.addData(">", "Press Play to start");
        telemetry.update();


        leftServo.setPosition(LEFT_CENTER);
        rightServo.setPosition(RIGHT_CENTER);

        waitForStart();
        ElapsedTime r = new ElapsedTime();



        relicTrackables.activate();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive() && (vuMark == RelicRecoveryVuMark.UNKNOWN) && runtime.seconds() <= 12){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark, ", "%s visible", vuMark);
            telemetry.update();
        }

        lift.setPower(UP_SPEED);

        s(800, runtime);
        lift.setPower(0);


        square_up(r);


        double timeToDrive;
        if (vuMark == RelicRecoveryVuMark.LEFT){
            encoderDrive(12);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT){
            encoderDrive(4);
        } else {
            encoderDrive(8);
        }

        encoderTurn(-56);
        encoderDrive(12);
        timeDrive(400, true, r);


        lift.setPower(DOWN_SPEED);
        s(800, r);
        lift.setPower(0);
        leftServo.setPosition(LEFT_SLIGHT_OUT);
        rightServo.setPosition(RIGHT_SLIGHT_OUT);
        s(250, r);

        encoderDrive(-12);
        leftServo.setPosition(LEFT_IN);
        rightServo.setPosition(RIGHT_IN);
        encoderDrive(10);
        timeDrive(200, true, r);
        encoderDrive(-10);
        encoderTurn(56);
        if (vuMark == RelicRecoveryVuMark.LEFT){
            encoderDrive(0);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            encoderDrive(4);
        } else {
            encoderDrive(8);
        }
    }


    public void s(double time, ElapsedTime r){
        double t = r.milliseconds();
        while (opModeIsActive() && r.milliseconds() <= t + time){}
    }

    public void encoderDrive(double inches){

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
                motor.setPower(0.25);
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

    public void encoderTurn(double degrees){

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
                motor.setPower(0.25);
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

    public void square_up(ElapsedTime r){
        timeDrive(1000, true, r);
        timePowerDrive(1000, false, 0.1, r);

        for (DcMotor motor : moreMotors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    
}
// one second per foot at 0.2 power