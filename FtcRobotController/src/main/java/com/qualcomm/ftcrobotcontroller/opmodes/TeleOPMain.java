/*
TeleOP Mode ver. 1.0
 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;


public class TeleOPMain extends OpMode
{
    //Declares motors
    DcMotor motor0;
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;

    //Declares motor controller
    DcMotorController.DeviceMode devMode;
    DcMotorController motorContL;
    DcMotorController motorContR;

    /*

        2---^---0
        |       |
        |       |
        3-------1

        Robot layout

     */

    public TeleOPMain()
    {

    }

    @Override
    public void init() {


		//Initializes motors to be referenced for the rest of the program.
        motor0 = hardwareMap.dcMotor.get("motor_0");
        motor1 = hardwareMap.dcMotor.get("motor_1");
        motor2 = hardwareMap.dcMotor.get("motor_2");
        motor3 = hardwareMap.dcMotor.get("motor_3");

        //Initializes motor controllers
        motorContL = hardwareMap.dcMotorController.get("motor_controller0");
        motorContR = hardwareMap.dcMotorController.get("motor_controller1");


    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1
		 *
		 *              Left joystick                               Right joystick
		 *                                         |
		 *                 moves up                |
		 *                   ^ ^                   |
		 *                  -----                  |                   -----
		 *                |       |                |                 |       |
		 *  moves left < |         | > moves right | rotates left < |         | > rotates right
		 *             < |         | >             |              < |         | >
		 *                |       |                |                 |       |
		 *                  -----                  |                   -----
		 *                   v v                   |
		 *                moves right              |
		 *
		 *
		 */

        //First handles rotation.
        float rotation = gamepad1.right_stick_x;
        rotation = Range.clip(rotation, -1, 1);

        //If push down on left joystick is greater than 10%, then it rotate the desired amount.
        if(rotation > 0.1 && rotation < -0.1)
        {
            motor0.setPower(rotation);
            motor1.setPower(rotation);
            motor2.setPower(rotation);
            motor3.setPower(rotation);
        }




        telemetry.addData("Text", "We runnin nerds");
        telemetry.addData("rot", "Rotation: " + String.format("%.2f",rotation));


    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }



}