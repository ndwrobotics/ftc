package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="3V1 Safe zone", group="test")
public class Auto3V1 extends LinearOpMode {
    Selectron bot = null;
    ElapsedTime r = null;
    @Override
    public void runOpMode() {
        bot = new Selectron(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        bot.resetJewelArm();
        r = new ElapsedTime();

        waitForStart();

        bot.lift.setPower(-bot.DOWN_SPEED);
        bot.s(800, r);
        bot.lift.setPower(0);
        bot.encoder4WDrive(36);
    }
}