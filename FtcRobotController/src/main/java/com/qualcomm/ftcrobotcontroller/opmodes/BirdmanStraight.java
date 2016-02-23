/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.DecimalFormat;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 * An example loop op mode where the robot will drive in
 * a straight line (where the driving direction is guided by
 * the Yaw angle from a navX-Model device).
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.  This example uses
 * the default update rate (50Hz), which may be lowered in order
 * to reduce the frequency of the updates to the drive system.
 */

public class BirdmanStraight extends OpMode {
    DcMotor motor0;
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motorRotate;

    ColorSensor sensorRGB;
    boolean ledOn;

    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;


    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;


    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;

    navXPIDController.PIDResult yawPIDResult;
    DecimalFormat df;

    /* Tune this threshold to adjust the sensitivy of the */
  /* Collision detection.                               */
    private final double COLLISION_THRESHOLD_DELTA_G = 1.25;

    double last_world_linear_accel_x;
    double last_world_linear_accel_y;


    private boolean collision_state;

    public boolean isOnWhite;
    public int state;


    @Override
    public void init() {
        motor0 = hardwareMap.dcMotor.get("motor_0");
        motor1 = hardwareMap.dcMotor.get("motor_1");
        motor2 = hardwareMap.dcMotor.get("motor_2");
        motor3 = hardwareMap.dcMotor.get("motor_3");
        motorRotate = hardwareMap.dcMotor.get("motor_4");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);


        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);


        df = new DecimalFormat("#.##");


        collision_state = false;

        sensorRGB = hardwareMap.colorSensor.get("mr");
        sensorRGB.enableLed(true);

        ledOn = true;
        isOnWhite = false;


        navx_device.zeroYaw();
        yawPIDResult = new navXPIDController.PIDResult();
    }


    @Override
    public void start() {


        runtime = new ElapsedTime();
        state = 0;
        last_world_linear_accel_x = 0.0;
        last_world_linear_accel_y = 0.0;
        while(runtime.time() < 5.0)
        {

        }
    }

    @Override
    public void loop() {

            double drive_speed = 0.5;
            if(state == 0)
            {
                if(runtime.time() < 10.0) {
                    if (!collision_state) {
                       // if(runtime.time() > 7.5)
                       // {
                        //    checkCollision();
                        //}

                        motor0.setPower(drive_speed);
                        motor1.setPower(drive_speed);
                        motor2.setPower(drive_speed);
                        motor3.setPower(drive_speed);



                    } else {


                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        state++;
                    }
                }
                else
                {
                    motor0.setPower(0);
                    motor1.setPower(0);
                    motor2.setPower(0);
                    motor3.setPower(0);
                    state++;
                }
            }
            else if(state == 1)
            {
                double startTime = runtime.time();
                while(!isOnWhite && (runtime.time()-startTime) < 0.5)
                {
                    whiteDetect();
                    motor0.setPower(-0.25);
                    motor1.setPower(-0.25);
                    motor2.setPower(-0.25);
                    motor3.setPower(-0.25);
                }
                state++;
            }
            else if(state == 2)
            {
                if (yawPIDController.isNewUpdateAvailable(yawPIDResult))
                {
                    double output = navx_device.getYaw();
                    if(!(output > -3 && output < 3))
                    {
                        motor1.setPower(0.25);
                        motor3.setPower(0.25);
                    }
                    else if(output > -3 && output < 3)
                    {
                        motor1.setPower(0);
                        motor3.setPower(0);
                        state++;
                    }

                }
            }
            else if(state == 3)
            {
                double startTime = runtime.time();
                int direction = 0;
                while(direction < 2)
                {
                        while(!isOnWhite && (runtime.time()-startTime) < 1.5)
                        {
                            whiteDetect();
                            motor0.setPower(-0.25);
                            motor1.setPower(-0.25);
                            motor2.setPower(-0.25);
                            motor3.setPower(-0.25);
                        }

                        while(!isOnWhite && ( runtime.time()-startTime) < 3)
                        {
                            whiteDetect();
                            motor0.setPower(0.25);
                            motor1.setPower(0.25);
                            motor2.setPower(0.25);
                            motor3.setPower(0.25);
                        }
                        if(isOnWhite)
                        {
                            break;
                        }
                        direction++;

                }
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                if(isOnWhite)
                {
                    state++;
                }
                else
                {
                    stop();
                }
            }
            else if(state == 4)
            {
                double startTime = runtime.time();
                while((runtime.time()-startTime) < 0.1)
                {

                    motor0.setPower(0.25);
                    motor1.setPower(0.25);
                    motor2.setPower(0.25);
                    motor3.setPower(0.25);
                }
                state++;
            }
            else if(state == 5) {
                if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                    double output = navx_device.getYaw();
                    if (!(output < -87 && output > -93)) {
                        motor0.setPower(0.25);
                        motor1.setPower(-0.25);
                        motor2.setPower(0.25);
                        motor3.setPower(-0.25);
                    } else if (output < -87 && output > -93) {
                        isOnWhite = false;
                        motor0.setPower(0);
                        motor1.setPower(0);
                        motor2.setPower(0);
                        motor3.setPower(0);
                        state++;
                    }
                }
            }
            else if(state == 6)
            {
                double startTime = runtime.time();
                while((runtime.time()-startTime) < 0.45)
                {

                    motor0.setPower(0.25);
                    motor1.setPower(0.25);
                    motor2.setPower(0.25);
                    motor3.setPower(0.25);
                }
                state++;
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
            }
            else if(state == 7)
            {
                double startTime = runtime.time();

                while((runtime.time()-startTime) < 3)
                {
                    motorRotate.setPower(-1);
                }
                state++;
            }
            else
            {

                motorRotate.setPower(0);
                motor0.setPower(0);
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                telemetry.addData("state:", "completed");
                stop();
            }
        telemetry.addData("state:", Integer.toString(state));
        telemetry.addData("collision:", Boolean.toString(collision_state));
        telemetry.addData("on white:", Boolean.toString(isOnWhite));
        if (yawPIDController.isNewUpdateAvailable(yawPIDResult))
        {
            double output = yawPIDResult.getOutput();
            telemetry.addData("current yaw:", Double.toString(output));
        }
    }

    @Override
    public void stop()
    {
        telemetry.addData("end state:", Integer.toString(state));
        navx_device.close();
        motor0.setPower(0);
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motorRotate.setPower(0);

    }


    public void checkCollision()
    {
        boolean collisionDetected = false;


        double curr_world_linear_accel_x = navx_device.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = navx_device.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;


        if ((Math.abs(currentJerkX) > COLLISION_THRESHOLD_DELTA_G) ||
                (Math.abs(currentJerkY) > COLLISION_THRESHOLD_DELTA_G)) {

            collisionDetected = true;
        }

        collision_state = collisionDetected;
    }

    public void whiteDetect()
    {
        double alphaValue = sensorRGB.alpha();
        double redValue = sensorRGB.red();
        double greenValue = sensorRGB.green();
        double blueValue = sensorRGB.blue();
        double totalValue = alphaValue + redValue + greenValue + blueValue;
        if(totalValue >= 3)
        {
             isOnWhite = true;
        }
        telemetry.addData("color total:", Double.toString(alphaValue+redValue+greenValue+blueValue));
    }
}
