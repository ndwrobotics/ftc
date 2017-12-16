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
public class SelectronV2
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

    double UP_SPEED = -0.5; //tweak this
    double DOWN_SPEED = 0.8;


    double LEFT_CENTER = 0.56; //tweak this
    double RIGHT_CENTER = 0.35; //tweak this
    double LEFT_SLIGHT_OUT = 0.47;
    double RIGHT_SLIGHT_OUT = 0.48;
    double LEFT_IN = 0.99; //tweak this
    double RIGHT_IN = 0.0; //tweak this
    double LEFT_OUT = 0.34; //tweak this
    double RIGHT_OUT = 0.53; //tweak this //done
    //--2-end

    int JEWEL_OUT = -580; //tweak this
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
    public SelectronV2(LinearOpMode opMode){
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
        if (count < 4 && op.opModeIsActive()) {
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
                    if (redAlliance){
                        encoderTurn(30, power);
                        encoderTurn(-30, power);
                    } else {
                        encoderTurn(-30, power);
                        encoderTurn(30, power);
                    }
                } else if (blue > 10 * color_threshold) {
                    if(redAlliance){
                        encoderTurn(-30, power);
                        encoderTurn(30, power);
                    } else {
                        encoderTurn(30, power);
                        encoderTurn(-30, power);
                    }

                } else {
                    if (count == 0){
                        encoderDrive(0.5, 0.15);
                    } else if (count == 1){
                        encoderDrive(-1, 0.15);
                    } else if (count == 2){
                        encoderDrive(-0.5, 0.15);
                    } else if (count == 3){
                        encoderDrive(-0.5, 0.15);
                    }
                    displaceJewel(color_threshold, count + 1, redAlliance);
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
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void placeGlyphVuforiaEtc(boolean red_alliance){
        ElapsedTime r = new ElapsedTime();

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
        if(vuMark == RelicRecoveryVuMark.LEFT){
            encoderDrive(0);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT){
            encoderDrive(8);
        } else {
            encoderDrive(4);
        }
    }

    public void square_up(ElapsedTime r, boolean redAlliance){

        if (redAlliance){
            time4WDrive(1200, true, r);
            timePowerDrive(1200, false, 0.2, r);
        } else {
            time4WDrive(1200, false, r);
            timePowerDrive(1200, true, 0.2, r);
        }
        s(250, r);
    }


    // None of the methods from here on have been extensively tested. Use at your own peril.
    public void encoderDrive(double inches, double power){
        for (DcMotor motor:someMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        int curPos[] = new int[4];
        for (int i = 0; i < 2; i++){
            curPos[i] = someMotors[i].getCurrentPosition();
        }
        if (inches > 0){
            wheelL.setPower(power);
            wheelR.setPower(power);
        } else {
            wheelL.setPower(-power);
            wheelR.setPower(-power);
        }
        boolean done = false;
        boolean wheelDone[] = new boolean[2];
        wheelDone[0] = false;
        wheelDone[1] = false;
        while (op.opModeIsActive() && !done){
            op.telemetry.addData("left position: ", wheelL.getCurrentPosition());
            op.telemetry.addData("left target: ", curPos[0] + counts);
            op.telemetry.addData("right position: ", wheelR.getCurrentPosition());
            op.telemetry.addData("right target: ", curPos[1] + counts);
            op.telemetry.update();
            for (int i = 0; i < 2; i++) {
                if (inches < 0) {
                    if (someMotors[i].getCurrentPosition() < curPos[i]+counts) {
                        someMotors[i].setPower(0);
                        wheelDone[i] = true;
                    }
                } else {
                    if (someMotors[i].getCurrentPosition() > curPos[i]+counts) {
                        someMotors[i].setPower(0);
                        wheelDone[i] = true;
                    }
                }
            }
            if(wheelDone[0] && wheelDone[1]){
                done = true;
            }
        }

        op.telemetry.addData("Left error:", wheelL.getCurrentPosition() - (curPos[0]+counts));
        op.telemetry.addData("Right error:", wheelR.getCurrentPosition() - (curPos[1]+counts));
        op.telemetry.update();
    }
    public void encoderDrive(double inches){
        encoderDrive(inches, 0.25);
    }
    public void encoderTurn(double degrees){
        encoderTurn(degrees, 0.25);
    }
    public void encoder4WDrive(double inches) { encoder4WDrive(inches, 0.25); }

    public void encoder4WDrive(double inches, double power){
        for (DcMotor motor:moreMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        int curPos[] = new int[4];
        for (int i = 0; i < 4; i++){
            curPos[i] = moreMotors[i].getCurrentPosition();
        }
        if (inches > 0){
            for (DcMotor motor:moreMotors){
                motor.setPower(power);
            }
        } else {
            for (DcMotor motor:moreMotors){
                motor.setPower(power);
            }
        }
        boolean done = false;
        boolean wheelDone[] = new boolean[4];
        wheelDone[0] = false;
        wheelDone[1] = false;
        wheelDone[3] = false;
        wheelDone[2] = false;
        while (op.opModeIsActive() && !done){
            op.telemetry.addData("left position: ", wheelL.getCurrentPosition());
            op.telemetry.addData("left target: ", curPos[0] + counts);
            op.telemetry.addData("right position: ", wheelR.getCurrentPosition());
            op.telemetry.addData("right target: ", curPos[1] + counts);
            op.telemetry.addData("left2 position: ", wheelL.getCurrentPosition());
            op.telemetry.addData("left2 target: ", curPos[2] + counts);
            op.telemetry.addData("right2 position: ", wheelR.getCurrentPosition());
            op.telemetry.addData("right2 target: ", curPos[3] + counts);
            op.telemetry.update();
            for (int i = 0; i < 4; i++) {
                if (inches < 0) {
                    if (moreMotors[i].getCurrentPosition() < curPos[i]+counts) {
                        moreMotors[i].setPower(0);
                        wheelDone[i] = true;
                    }
                } else {
                    if (moreMotors[i].getCurrentPosition() > curPos[i]+counts) {
                        moreMotors[i].setPower(0);
                        wheelDone[i] = true;
                    }
                }
            }
            if(wheelDone[0] && wheelDone[1] && wheelDone[2] && wheelDone[3]){
                done = true;
            }
        }

        op.telemetry.addData("Left error:", wheelL.getCurrentPosition() - (curPos[0]+counts));
        op.telemetry.addData("Right error:", wheelR.getCurrentPosition() - (curPos[1]+counts));
        op.telemetry.addData("Left2 error:", wheelL2.getCurrentPosition() - (curPos[2]+counts));
        op.telemetry.addData("Right2 error:", wheelR2.getCurrentPosition() - (curPos[3]+counts));
        op.telemetry.update();
    }

    public void encoderTurn(double degrees, double power){
        for (DcMotor motor:someMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 17.0/16.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        int curPos[] = new int[4];
        for (int i = 0; i < 2; i++){
            curPos[i] = someMotors[i].getCurrentPosition();
        }
        if (degrees > 0){
            wheelL.setPower(-power);
            wheelR.setPower(power);
        } else {
            wheelL.setPower(power);
            wheelR.setPower(-power);
        }

        boolean rDone = false;
        boolean lDone = false;
        while (op.opModeIsActive() && (!rDone || !lDone)){
            op.telemetry.addData("left position: ", wheelL.getCurrentPosition());
            op.telemetry.addData("left target: ", curPos[0] - counts);
            op.telemetry.addData("right position: ", wheelR.getCurrentPosition());
            op.telemetry.addData("right target: ", curPos[1] + counts);
            op.telemetry.update();
            if (degrees < 0) {
                if (wheelR.getCurrentPosition() < curPos[1]+counts) {
                    wheelR.setPower(0);
                    rDone = true;
                }
                if (wheelL.getCurrentPosition() > curPos[0]-counts) {
                    wheelL.setPower(0);
                    lDone = true;
                }
            } else {
                if (wheelR.getCurrentPosition() > curPos[1]+counts) {
                    wheelR.setPower(0);
                    rDone = true;
                }
                if (wheelL.getCurrentPosition() < curPos[0]-counts) {
                    wheelL.setPower(0);
                    lDone = true;
                }
            }
        }

        op.telemetry.addData("Left error:", wheelL.getCurrentPosition() - (curPos[0]-counts));
        op.telemetry.addData("Right error:", wheelR.getCurrentPosition() - (curPos[1]+counts));
        op.telemetry.update();
    }


    /*public void newAndImprovedEncoderDrive(double inches, double power, int stuckCount){
        ElapsedTime r = new ElapsedTime();
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        int curPosL = wheelL.getCurrentPosition();
        int curPosR = wheelR.getCurrentPosition();

        if (op.opModeIsActive()) {
            for (DcMotor motor : someMotors) {
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : someMotors) {
                motor.setPower(0.25);
            }

            int stuckCode = 0;
            boolean stuck = false;
            while (op.opModeIsActive() && !stuck && (wheelL.isBusy() || wheelR.isBusy())) {
                stuckCode = isStuck(stuckCode);
                if (stuckCode > 2){
                    stuck = true;
                }
                if (stuck){
                    op.telemetry.addLine("STUCK!!!!");
                } else {
                    op.telemetry.addData("Target position: ", inches);
                    op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                    op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                    op.telemetry.update();
                }
                op.telemetry.update();
            }
            for (DcMotor motor : someMotors) {
                motor.setPower(0);
            }
            if (stuck){
                op.telemetry.addData("Stuck count: ", stuckCount+1);
                op.telemetry.update();
                wheelL.setTargetPosition(wheelR.getCurrentPosition() - curPosR + curPosL);
                wheelL.setPower(0.25);
                while(wheelL.isBusy()){}
                wheelL.setPower(0);
                s(250, r);
                int currentError = curPosL + counts - wheelL.getCurrentPosition();
                newAndImprovedEncoderDrive(currentError/COUNTS_PER_INCH, 0.25, stuckCount+1);
            } else {
                op.telemetry.addData("Stuck count: ", stuckCount);
                op.telemetry.update();
            }
            s(250, r);
        }
    }
    public void newAndImprovedEncoderDrive(double degrees, double power){
        newAndImprovedEncoderDrive(degrees, power, 0);
    }

    public void newAndImprovedEncoderTurn(double degrees, double power, int stuckCount){
        ElapsedTime r = new ElapsedTime();
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 17.0/16.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        int curPosL = wheelL.getCurrentPosition();
        int curPosR = wheelR.getCurrentPosition();

        if (op.opModeIsActive()) {
            wheelL.setTargetPosition(wheelL.getCurrentPosition() - counts);
            wheelR.setTargetPosition(wheelR.getCurrentPosition() + counts);
            for (DcMotor motor : someMotors) {
                motor.setPower(0.25);
            }

            int stuckCode = 0;
            boolean stuck = false;
            while (op.opModeIsActive() && !stuck && (wheelL.isBusy() || wheelR.isBusy())) {
                stuckCode = isStuck(stuckCode);
                if (stuckCode > 2){
                    stuck = true;
                }
                if (stuck){
                    op.telemetry.addLine("STUCK!!!!");
                } else {
                    op.telemetry.addData("Target position: ", degrees);
                    op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                    op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                    op.telemetry.update();
                }
                op.telemetry.update();
            }
            for (DcMotor motor : someMotors) {
                motor.setPower(0);
            }
            if (stuck){
                op.telemetry.addData("Stuck count: ", stuckCount+1);
                op.telemetry.update();
                wheelL.setTargetPosition(wheelR.getCurrentPosition() - curPosR + curPosL);
                wheelL.setPower(0.25);
                while(wheelL.isBusy()){}
                wheelL.setPower(0);
                s(250, r);
                int currentError = curPosL + counts - wheelL.getCurrentPosition();
                newAndImprovedEncoderTurn(currentError/COUNTS_PER_DEGREE, 0.25, stuckCount+1);
            } else {
                op.telemetry.addData("Stuck count: ", stuckCount);
                op.telemetry.update();
            }
            s(250, r);
        }
    }
    public void newAndImprovedEncoderTurn(double degrees, double power){
        newAndImprovedEncoderTurn(degrees, power, 0);
    }

    public int isStuck(int prevCode){
        int newCode;

        if (!atTarget(wheelL) || !atTarget(wheelR)) {
            int curPosL = wheelL.getCurrentPosition();
            int curPosR = wheelR.getCurrentPosition();
            ElapsedTime r = new ElapsedTime();
            while (op.opModeIsActive() && r.milliseconds() < 100) {
            }
            if (((Math.abs(wheelL.getCurrentPosition() - curPosL) < 10) && !atTarget(wheelL)) || ((Math.abs(wheelR.getCurrentPosition() - curPosR) < 10) && !atTarget(wheelR))) {
                newCode = prevCode + 1;
            } else {
                newCode = 0;
            }
        } else {
            newCode = 0;
        }
        return newCode;
    }

    public boolean atTarget(DcMotor motor){
        if (Math.abs(motor.getTargetPosition()-motor.getCurrentPosition()) < 10){
            return true;
        } else {
            return false;
        }
    }*/
 }









