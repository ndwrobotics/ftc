package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


@Autonomous(name="4V4 -Blue Not Mat", group="Autonomous")
public class Auto4V4BlueNotMat extends LinearOpMode {
    SelectronV3 bot = null;
    boolean RED_ALLIANCE = false;

    @Override
    public void runOpMode() {
        bot = new SelectronV3(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        bot.resetJewelArm();
        bot.initVuforia();

        ElapsedTime r = new ElapsedTime();

        waitForStart();

        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
        bot.jewelDisplacer.setPower(0.25);

        bot.findPicture();

        bot.autoRaiseLift();

        bot.displaceJewel(bot.COLOR_THRESHOLD, RED_ALLIANCE);

        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_IN);

        bot.square_up(r, RED_ALLIANCE);



        if(bot.vuMark == RelicRecoveryVuMark.LEFT){
            bot.encoderDrive(-8);
            bot.encoderTurn(157);
            bot.encoderDrive(24);
        } else if (bot.vuMark == RelicRecoveryVuMark.RIGHT){
            bot.encoderDrive(-5);
            bot.encoderTurn(135);
            bot.encoderDrive(6);

        } else {
            bot.encoderDrive(-5);
            bot.encoderTurn(145);
            bot.encoderDrive(8);
        }
        bot.timeDrive(600, true, r);
        bot.release();
        bot.s(250, r);
        bot.encoderDrive(-8);
        bot.autoLowerLift();
        bot.completely_close();
        bot.encoderDrive(8);
        bot.encoderDrive(-10);

        if(bot.vuMark == RelicRecoveryVuMark.LEFT){
            bot.encoderTurn(-65);
            bot.encoderDrive(8);
        } else if (bot.vuMark == RelicRecoveryVuMark.RIGHT){
            bot.encoderTurn(-30);
        } else {
            bot.encoderTurn(-43);
            bot.encoderDrive(4);
        }

    }
}