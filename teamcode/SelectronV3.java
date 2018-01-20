package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * Is also defines all of the methods for the robot.
 * In that case, this is our (10925 Spartacles') robot.
 * (Affectionately dubbed Selectron.)
 */
public class SelectronV3
{
    /* Public OpMode members. */
    //--1-begin
    DcMotor wheelR = null;
    DcMotor wheelL = null;
    //--1-end
    DcMotor wheelR2 = null;
    DcMotor wheelL2 = null;

    //--2-begin
    Servo leftServo = null;
    Servo rightServo = null;
    //--2-end

    DcMotor jewelDisplacer = null;

    TouchSensor touchSensor = null;

    DcMotor lift = null;

    NormalizedColorSensor colorSensor = null;
    DcMotor someMotors[] = null;
    DcMotor moreMotors[] = null;

    double UP_SPEED = -0.7; //tweak this
    double DOWN_SPEED = 0.8;


    double LEFT_CENTER = 0.59; //tweak this
    double RIGHT_CENTER = 0.3; //tweak this
    double LEFT_SLIGHT_OUT = 0.47;
    double RIGHT_SLIGHT_OUT = 0.46;
    double LEFT_IN = 0.99; //tweak this
    double RIGHT_IN = 0.0; //tweak this
    double LEFT_OUT = 0.34; //tweak this
    double RIGHT_OUT = 0.51; //tweak this //done
    //--2-end

    int JEWEL_OUT = -620; //tweak this
    int JEWEL_IN = 0; //tweak this //done

    double COLOR_THRESHOLD = 0.71;



    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark vuMark = null;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;




    /* local OpMode members. */
    LinearOpMode op = null;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SelectronV3(LinearOpMode opMode){
        op = opMode;

        colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, "color");

        //--1-begin
        wheelR = op.hardwareMap.dcMotor.get("motorR");
        wheelL = op.hardwareMap.dcMotor.get("motorL");
        //--1-begin
        wheelR2 = op.hardwareMap.dcMotor.get("motorR2");
        wheelL2 = op.hardwareMap.dcMotor.get("motorL2");

        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        wheelL2.setDirection(DcMotorSimple.Direction.REVERSE);

        touchSensor = op.hardwareMap.touchSensor.get("touchUp");


        leftServo = op.hardwareMap.servo.get("left");
        rightServo = op.hardwareMap.servo.get("right");

        lift = op.hardwareMap.dcMotor.get("lift");

        jewelDisplacer = op.hardwareMap.dcMotor.get("jeweler");


        someMotors = new DcMotor[2];
        moreMotors = new DcMotor[4];

        someMotors[0] = wheelL;
        someMotors[1] = wheelR;

        moreMotors[0] = wheelL;
        moreMotors[1] = wheelR;
        moreMotors[2] = wheelL2;
        moreMotors[3] = wheelR2;
        for (DcMotor motor: moreMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        jewelDisplacer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftServo.setPosition(LEFT_CENTER);
        rightServo.setPosition(RIGHT_CENTER);
    }


    public void resetJewelArm(){
        jewelDisplacer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jewelDisplacer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initVuforia(){
        op.telemetry.addLine("Initializing Vuforia...");
        op.telemetry.update();
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "Aczwc7j/////AAAAGSh6oYrgWE/wvIHsaTUy6ppNpIawgvZu5xiQAUK+xtcW8Zw8P9GhxqBOsB2RKBAP6cOUkwCiyhCaYHKfrhA7ORLNRka9TTTm36bBHMY/WhR02K5Z5Qa2TaqRNjtpZ5rZ2Q8Ee+vnuiVasqj3uAMCB0ceFPYemYJ8snub+w/8gKVwy09n+ZWJ/5yymbJ9lGIQAwlc8Wo0HogRSK6Yk0Z0CiX/UFKhB+wK+I7Vzcwv0pZVOXAhf7fuDJOUr+TcuCBptm6AlFai2evVFuKT1seKaTsX+Sa2jvP3F0O1gbfLYK1dWjIh5/mqEp6yWgk2D+fAhGGWVjeUkN0ftstV6F1U06+f9P52sOd6w0Fuagpm27TP";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        op.telemetry.addLine("Vuforia Initialized. Press Play to Start.");
        op.telemetry.update();
    }

    public void findPicture(){
        relicTrackables.activate();
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        ElapsedTime runtime = new ElapsedTime();
        while (op.opModeIsActive() && (vuMark == RelicRecoveryVuMark.UNKNOWN) && runtime.seconds() <= 7){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            op.telemetry.addData("VuMark, ", "%s visible", vuMark);
            op.telemetry.update();
        }
    }


    public void setEncoders(DcMotor.RunMode mode){
        for (DcMotor motor:moreMotors){
            motor.setMode(mode);
        }
    }


    public void displaceJewel(double color_threshold, int count, boolean redAlliance) {
        if (count < 5 && op.opModeIsActive()) {
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
                op.telemetry.addLine("normalized color:  ")
                        .addData("a", colors.alpha)
                        .addData("r", colors.red)
                        .addData("g", colors.green)
                        .addData("b", colors.blue);
                op.telemetry.update();
                s(50, r);
            }
            op.telemetry.addData("blue: ", blue);
            op.telemetry.addData("red: ", red);
            op.telemetry.update();

            double power = 0.15;

            if (op.opModeIsActive()){
                if (red > 10 * color_threshold) {
                    if (blue > 10 * color_threshold) {
                        jewelDisplacer.setTargetPosition(JEWEL_OUT-5);

                    } else {
                        if (redAlliance) {
                            encoderPowerTurn(30, power);
                            encoderPowerTurn(-30, power);
                        } else {
                            encoderPowerTurn(-30, power);
                            encoderPowerTurn(30, power);
                        }
                    }
                } else {
                    if (blue > 10 * color_threshold){
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
                            encoderPowerDrive(0.3, 0.15);
                        } else if (count == 2){
                            encoderPowerDrive(-1.2, 0.15);
                            s(250, r);
                        } else if (count == 3){
                            encoderPowerDrive(-0.5, 0.15);
                        } else if (count == 4){
                            encoderPowerDrive(-0.5, 0.15);
                        }
                        displaceJewel(color_threshold, count + 1, redAlliance);
                    }
                }
            }
        }
        jewelDisplacer.setTargetPosition(JEWEL_IN);
    }

    public void displaceJewel(double color_threshold, boolean redAlliance){
        displaceJewel(color_threshold, 0, redAlliance);
    }


    public void s(double time, ElapsedTime r){
        double t = r.milliseconds();
        while (op.opModeIsActive() && r.milliseconds() <= t + time){}
    }

    public void encoderPowerDrive(double inches, double power){
        newEncoderInchesPowerDrive(inches, power, getInchesTimeout(inches));
    }

    public void encoderDrive(double inches){
        encoderPowerDrive(inches, 0.35);
    }


    public void encoder4WDrive(double inches){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);

        if (op.opModeIsActive()) {
            for (DcMotor motor : moreMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0.25);
            }
            while (op.opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy() || wheelL2.isBusy() || wheelR2.isBusy())){
                op.telemetry.addData("Target position: ", inches);
                op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                op.telemetry.update();
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }

    public void encoderPowerTurn(double degrees, double power){
        newEncoderDegreesPowerTurn(degrees, power, getDegreesTimeout(degrees));
    }

    public void encoderTurn(double degrees){
        encoderPowerTurn(degrees, 0.25);
    }

    public void timePowerDrive(double milliseconds, boolean forwards, double power, ElapsedTime r){
        for (DcMotor motor : someMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotor motor : someMotors){
            if (forwards){
                motor.setPower(power);
            } else {
                motor.setPower(-power);
            }
        }
        s(milliseconds, r);
        for (DcMotor motor : someMotors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        s(250, r);
    }

    public void timeDrive(double milliseconds, boolean forwards, ElapsedTime r){
        timePowerDrive(milliseconds, forwards, 0.25, r);
    }

    public void timePower4WDrive(double milliseconds, boolean forwards, double power, ElapsedTime r){
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
    public void time4WDrive(double milliseconds, boolean forwards, ElapsedTime r){
        timePower4WDrive(milliseconds, forwards, 0.25, r);
    }

    public void placeGlyphEtc(boolean red_alliance){
        ElapsedTime r = new ElapsedTime();
        if (red_alliance) {
            encoderDrive(10.5);
        } else {
            encoderDrive(-25);// -17.5 or -25 or -32.5
        }

        encoderTurn(-60);
        encoderDrive(12);
        timeDrive(400, true, r);


        if (op.opModeIsActive()){
            lift.setPower(DOWN_SPEED);
        }
        s(800, r);
        lift.setPower(0);
        leftServo.setPosition(LEFT_SLIGHT_OUT);
        rightServo.setPosition(RIGHT_SLIGHT_OUT);
        s(250, r);
        encoderDrive(-12);
        leftServo.setPosition(LEFT_IN);
        rightServo.setPosition(RIGHT_IN);
        encoderDrive(10);
        timeDrive(400, true, r);
        encoderDrive(-10);
        encoderTurn(60);
        encoderDrive(4); // 0 or 4 or 8?
    }

    public void partPlaceGlyphVuforiaEtc(boolean red_alliance){
        ElapsedTime r = new ElapsedTime();

        encoderTurn(-60);
        encoderDrive(12);
        timeDrive(400, true, r);

        autoLowerLift();

        leftServo.setPosition(LEFT_SLIGHT_OUT);
        rightServo.setPosition(RIGHT_SLIGHT_OUT);
        s(250, r);
        encoderDrive(-12);
        leftServo.setPosition(LEFT_IN);
        rightServo.setPosition(RIGHT_IN);
        encoderDrive(10);
        timeDrive(400, true, r);
        encoderDrive(-10);

    }
    public void placeGlyphVuforiaEtc(boolean red_alliance){
        partPlaceGlyphVuforiaEtc(red_alliance);
        encoderTurn(60);
        if(vuMark == RelicRecoveryVuMark.LEFT){
            encoderDrive(0);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT){
            encoderDrive(8);
        } else {
            encoderDrive(4);
        }
    }

    public void specialPlaceGlyphVuforiaEtc(boolean red_alliance){
        partPlaceGlyphVuforiaEtc(red_alliance);
        encoderTurn(-60);
        if(vuMark == RelicRecoveryVuMark.LEFT){
            encoderDrive(8);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT){
            encoderDrive(0);
        } else {
            encoderDrive(4);
        }

    }

    public void square_up(ElapsedTime r, boolean redAlliance){

        if (redAlliance){
            time4WDrive(1200, true, r);
            timePowerDrive(1000, false, 0.3, r);
        } else {
            time4WDrive(1200, false, r);
            timePowerDrive(1000, true, 0.3, r);
        }
        s(250, r);
    }


    public void newEncoderInchesPowerDrive(double inches, double power, double timeout){
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        newEncoderPowerDrive(counts, power, timeout);
    }
    // None of the methods from here on have been extensively tested. Use at your own peril.
    public void newEncoderPowerDrive(int encoderCounts, double power, double timeout){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch

        boolean timedOut = false;


        int leftPos = wheelL.getCurrentPosition();
        int rightPos = wheelR.getCurrentPosition();
        if (op.opModeIsActive()) {
            for (DcMotor motor : someMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + encoderCounts);
            }
            ElapsedTime r = new ElapsedTime();
            for (DcMotor motor : someMotors){
                motor.setPower(power);
            }
            while (op.opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy()) && !timedOut){
                if(r.milliseconds() > timeout){
                    timedOut = true;
                }
                op.telemetry.addData("Target position Left: ", leftPos + encoderCounts);
                op.telemetry.addData("Target position right: ", rightPos + encoderCounts);
                op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                op.telemetry.update();
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }

            op.telemetry.addData("Time: ", r.milliseconds());
            op.telemetry.update();

            s(250, new ElapsedTime());
            if(timedOut){
                int rightError = rightPos + encoderCounts - wheelR.getCurrentPosition();
                int leftError = leftPos + encoderCounts - wheelL.getCurrentPosition();
                int drive = (rightError + leftError)/2;
                newEncoderPowerDrive(drive, power, timeout);
            }

        }
    }


    public void newEncoderDegreesPowerTurn(double degrees, double power, double timeout){
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 17.0/16.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        newEncoderPowerTurn(counts, power, timeout);
    }


    public void newEncoderPowerTurn(int encoderCounts, double power, double timeout){
        // degrees positive goes counterclockwise
        // degrees negative goes clockwise

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 280/pi counts per inch
        // turn diameter is 13.75 inches, so a 360 degree turn is 13.75pi inches
        // so one degree is 13.75pi/360 = 55pi/1440 = 11pi/288
        // 280/pi counts per inch times 11pi/288 inches per degree is 11*280/288 counts per degree
        // 11*280/288 = 11*35/36 = 385/36

        if (op.opModeIsActive()) {
            wheelL.setTargetPosition(wheelL.getCurrentPosition() - encoderCounts);
            wheelR.setTargetPosition(wheelR.getCurrentPosition() + encoderCounts);
            int leftPos = wheelL.getCurrentPosition();
            int rightPos = wheelR.getCurrentPosition();
            boolean timedOut = false;
            ElapsedTime r = new ElapsedTime();
            // should go counterclockwise
            for (DcMotor motor : someMotors){
                motor.setPower(power);
            }
            while (op.opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy()) && !timedOut){
                if(r.milliseconds() > timeout){
                    timedOut = true;
                }
                op.telemetry.addData("Target position: ", encoderCounts);
                op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                op.telemetry.update();
            }

            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }

            if(timedOut){
                int rightError = rightPos + encoderCounts - wheelR.getCurrentPosition();
                int leftError = wheelL.getCurrentPosition() - (leftPos - encoderCounts);
                int turn = (rightError + leftError)/2;
                newEncoderPowerTurn(turn, power, timeout);
            }
            op.telemetry.addData("Time: ", r.milliseconds());
            op.telemetry.update();
            s(250, r);
        }
    }


    public void newEncoderPowerInchesAutoTimeoutDrive(double inches){
        newEncoderInchesPowerDrive(inches, 0.35, getInchesTimeout(inches));
    }


    public double getInchesTimeout(double inches){
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        return getCountsTimeout(counts);
    }

    public double getCountsTimeout(int counts){
        double a = 0.8; //tweak this
        double b = 400; //tweak this
        return a*Math.abs(counts) + b;
    }

    public double getDegreesTimeout(double degrees){
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 17.0/16.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        return getCountsTimeout(counts);
    }




    public void autoRaiseLift(){
        if(op.opModeIsActive()){
            lift.setPower(-DOWN_SPEED);
        }

        s(800, new ElapsedTime());
        lift.setPower(0);
    }
    public void autoLowerLift(){
        if(op.opModeIsActive()){
            lift.setPower(DOWN_SPEED);
        }

        s(800, new ElapsedTime());
        lift.setPower(0);
    }


    public void release(){
        leftServo.setPosition(LEFT_OUT);
        rightServo.setPosition(RIGHT_OUT);
    }
    public void grab(){
        leftServo.setPosition(LEFT_CENTER);
        rightServo.setPosition(RIGHT_CENTER);
    }
    public void slight_release(){
        leftServo.setPosition(LEFT_SLIGHT_OUT);
        rightServo.setPosition(RIGHT_SLIGHT_OUT);
    }
    public void completely_close(){
        leftServo.setPosition(LEFT_IN);
        rightServo.setPosition(RIGHT_IN);
    }
}
//