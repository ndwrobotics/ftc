package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="3V2 Glyph -Red Mat", group="Autonomous")
//@Disabled
public class Auto3V2RedMat extends LinearOpMode
{


    Selectron bot = null;
    boolean RED_ALLIANCE = true;

    @Override
    public void runOpMode() {

        bot = new Selectron(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        ElapsedTime r = new ElapsedTime();

        if(opModeIsActive()){
            bot.lift.setPower(-bot.DOWN_SPEED);
        }

        bot.s(800, r);
        bot.lift.setPower(0);

        bot.square_up(r, RED_ALLIANCE);

        bot.encoderDrive(10.5);// 3 or 10.5 or 18
        bot.encoderTurn(-60);
        bot.encoderDrive(12);
        bot.timeDrive(400, true, r);


        if (opModeIsActive()){
            bot.lift.setPower(bot.DOWN_SPEED);
        }
        bot.s(800, r);
        bot.lift.setPower(0);
        bot.leftServo.setPosition(bot.LEFT_SLIGHT_OUT);
        bot.rightServo.setPosition(bot.RIGHT_SLIGHT_OUT);
        bot.s(250, r);
        bot.encoderDrive(-12);
        bot.leftServo.setPosition(bot.LEFT_IN);
        bot.rightServo.setPosition(bot.RIGHT_IN);
        bot.encoderDrive(10);
        bot.timeDrive(400, true, r);
        bot.encoderDrive(-10);
        bot.encoderTurn(60);
        bot.encoderDrive(4);

    }
}