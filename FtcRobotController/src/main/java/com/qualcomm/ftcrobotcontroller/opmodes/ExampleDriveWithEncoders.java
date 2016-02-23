package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class ExampleDriveWithEncoders extends LinearOpMode {

    DcMotor frontMotor;
    DcMotor backMotor;

    final static int ENCODER_CPR = 1440;     //Encoder Counts per Revolution
    final static double GEAR_RATIO = 2;      //Gear Ratio
    final static int WHEEL_DIAMETER = 4;     //Diameter of the wheel in inches
    final static int DISTANCE = 24;          //Distance in inches to drive

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;
    final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;

    DcMotorController wheelControllerFront;
    DcMotorController wheelControllerBack;

    @Override
    public void runOpMode() {
        frontMotor = hardwareMap.dcMotor.get("motor_0");
        backMotor = hardwareMap.dcMotor.get("motor_1");

        frontMotor.setDirection(DcMotor.Direction.REVERSE);

        frontMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        wheelControllerFront = hardwareMap.dcMotorController.get("wheelControllerFront");
        wheelControllerBack = hardwareMap.dcMotorController.get("wheelControllerBack");

        frontMotor.setTargetPosition((int) 1000);
        backMotor.setTargetPosition((int) 1000);

        frontMotor.setPower(0.3);
        backMotor.setPower(0.3);

        frontMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        frontMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        backMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        frontMotor.setTargetPosition((int) 2000);
        backMotor.setTargetPosition((int) 2000);

        frontMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        backMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);


    }
}

