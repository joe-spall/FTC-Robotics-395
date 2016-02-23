/* Copyright (c) 2015 Qualcomm Technologies Inc

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




import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.

 *
 * The op mode assumes that the color sensor
 * is configured with a name of "mr".
 *
 * You can use the X button on either gamepad to turn the LED on and off.
 *
 */
public class MRRGBExample extends OpMode {

  ColorSensor sensorRGB;
  boolean ledOn;


  @Override
  public void init()
  {
    sensorRGB = hardwareMap.colorSensor.get("mr");
    sensorRGB.enableLed(true);
    ledOn = true;

  }
  @Override
  public void loop() {



    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;


    // bPrevState and bCurrState represent the previous and current state of the button.

    boolean xState = gamepad1.x;
    boolean yState = gamepad1.y;

    if(xState) {
      ledOn = true;
    }

    if(yState) {
      ledOn = false;
    }

    sensorRGB.enableLed(ledOn);






      // send the info back to driver station using telemetry function.
      telemetry.addData("Clear", sensorRGB.alpha());
      telemetry.addData("Red  ", sensorRGB.red());
      telemetry.addData("Green", sensorRGB.green());
      telemetry.addData("Blue ", sensorRGB.blue());


      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.



    }
  }

