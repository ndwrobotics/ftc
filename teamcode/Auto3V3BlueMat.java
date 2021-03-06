package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="3V3 Vuforia -Blue Mat", group="Autonomous")
//@Disabled
public class Auto3V3BlueMat extends LinearOpMode
{


    Selectron bot = null;
    boolean RED_ALLIANCE = false;

    @Override
    public void runOpMode() {

        bot = new Selectron(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        bot.resetJewelArm();
        bot.initVuforia();

        waitForStart();
        ElapsedTime r = new ElapsedTime();

        bot.findPicture();

        if(opModeIsActive()){
            bot.lift.setPower(-bot.DOWN_SPEED);
        }

        bot.s(800, r);
        bot.lift.setPower(0);

        bot.square_up(r, RED_ALLIANCE);

        if(bot.vuMark == RelicRecoveryVuMark.LEFT){
            bot.encoderDrive(-17.5);
        } else if (bot.vuMark == RelicRecoveryVuMark.RIGHT){
            bot.encoderDrive(-32.5);
        } else {
            bot.encoderDrive(-25);
        }

        bot.placeGlyphVuforiaEtc(RED_ALLIANCE);

    }
}