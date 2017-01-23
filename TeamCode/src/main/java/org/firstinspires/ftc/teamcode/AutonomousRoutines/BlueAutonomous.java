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

package org.firstinspires.ftc.teamcode.AutonomousRoutines;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Blue Autonomous", group = "Auton")
//@Disabled                            // Comment this out to add to the opmode list
public class BlueAutonomous extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor ultrasonic;
    ColorSensor beaconSensor, lineSensor;
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor Shooter;
    DcMotor Harvester;
    Servo beaconPusher;

    double ticsPerInch = 38.5;
    double strafeTicsPerInch = 53;

    double leftPos = .8;
    double rightPos = .3;

    //sets default Powers
    double shooterPower = 1;
    double harvesterPower = 1;
    int shooterRot = 1625;
    @Override
    public void runOpMode() throws InterruptedException {

        Shooter = hardwareMap.dcMotor.get("Sh");
        Harvester = hardwareMap.dcMotor.get("Ha");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        ultrasonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultra");
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        beaconPusher = hardwareMap.servo.get("beaconPush");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        Harvester.setDirection(DcMotor.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beaconSensor = hardwareMap.colorSensor.get("beaconSense");
        lineSensor = hardwareMap.colorSensor.get("lineSense");
        lineSensor.setI2cAddress(I2cAddr.create8bit(0x4c));
        beaconSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconPusher.setPosition(0);
        gyro.calibrate();
        while(gyro.isCalibrating()){
            idle();
        }
        beaconSensor.enableLed(false);
        lineSensor.enableLed(true);
        waitForStart();
        shootBalls();
        gyro.resetZAxisIntegrator();
        moveRightInches(33,0.8);//initial strafe off wall
        Thread.sleep(100);
        pointToZero();//reorient straight
        Thread.sleep(100);
        moveBackwardInches(36,0.5);//reverse towards ramp
        Thread.sleep(500);
        moveRightInches(6,0.7);//Move past ramp
        Thread.sleep(100);
        pointToZero();//align to 0 to take reading
        startBackward(0.2);
        while(ultrasonic.getDistance(DistanceUnit.INCH) > 12){
            idle();
        }
        stopDrive();
        startRight(0.6);//move to white line
        int floorReading = lineSensor.green();
        while(opModeIsActive() && lineSensor.green() <= floorReading){
            if(ultrasonic.getDistance(DistanceUnit.INCH) > 15) {
                startBackward(0.3);
            }
            else
                startRight(0.6);
            idle();
        }
        stopDrive();
        Thread.sleep(100);
        moveLeftInches(2,0.6);//re-center on beacon
        pointToZero();//re-align to zero
        Thread.sleep(100);
        double wallDist = ultrasonic.getDistance(DistanceUnit.INCH);//store distance from wall
        if(wallDist < 6)//if too close
            moveForwardInches(6-wallDist,0.3);//move away
        else if(wallDist > 6)//if too far
            moveBackwardInches(wallDist-6,0.3);//move in
        pointToZero();
        pushBeaconBlue();//Press beacon after making decision of side
        pointToZero();//realign to zero
        moveForwardInches(10,0.2);
        Thread.sleep(250);
        beaconPusher.setPosition(1);
        moveRightInches(36,1);//move towards second beacon
        pointToZero();//align to 0 to take reading
        startBackward(0.2);
        while(ultrasonic.getDistance(DistanceUnit.INCH) > 11){
            idle();
        }
        stopDrive();
        startRight(0.6);//move to second white line
        floorReading = lineSensor.green();
        while(opModeIsActive() && lineSensor.green() <= floorReading){
            idle();
        }
        stopDrive();
        Thread.sleep(100);
        moveLeftInches(3,0.6);//recenter on white line
        pointToZero();//realign to zero
        Thread.sleep(100);
        wallDist = ultrasonic.getDistance(DistanceUnit.INCH);//store distance from wall
        if(wallDist < 6)//if too close
            moveForwardInches(6-wallDist,0.3);//move away
        else if(wallDist > 6)//if too far
            moveBackwardInches(wallDist-6,0.3);//move in
        pushBeaconBlue();//press second beacon
        moveForwardInches(5,1);

    }

    public void pushBeaconBlue() throws  InterruptedException{
        if(beaconSensor.blue() > beaconSensor.red()) {
            beaconPusher.setPosition(leftPos);
            Thread.sleep(500);
        }
        else {
            beaconPusher.setPosition(rightPos);
            Thread.sleep(500);
        }
        runtime.reset();
        startBackward(0.2);
        while(opModeIsActive() && runtime.seconds() < 1.5) {
            idle();
        }
    }

    public void pointToZero(){
        double angle = gyro.getIntegratedZValue();
        if(angle > 0)//If we're pointed left, do right turn
            rightTurn(angle,0.25);
        else if(angle < 0)//If we're pointed right, do left turn
            leftTurn(Math.abs(angle),0.25);
    }

    public void shootBalls(){
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setPower(shooterPower);
        while(opModeIsActive() && Shooter.getCurrentPosition() < shooterRot){
            int pos = Shooter.getCurrentPosition();
            if(pos > 50 && pos < shooterRot - 25)
                Harvester.setPower(harvesterPower);
            else
                Harvester.setPower(0);
        }
        Shooter.setPower(0);
        Harvester.setPower(harvesterPower);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            idle();
        }
        Harvester.setPower(0);
        Shooter.setPower(shooterPower);
        while(opModeIsActive() && Shooter.getCurrentPosition() < shooterRot*2){
            int pos = Shooter.getCurrentPosition();
            if(pos > shooterRot+50 && pos < shooterRot*2 - 25)
                Harvester.setPower(harvesterPower);
            else
                Harvester.setPower(0);
        }
        Shooter.setPower(0);
        Harvester.setPower(harvesterPower);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            idle();
        }
        Shooter.setPower(shooterPower);
        while(opModeIsActive() && Shooter.getCurrentPosition() < shooterRot*2+1000){
            idle();
        }
        Harvester.setPower(0);
        Shooter.setPower(0);
    }

    public void startForward(double speed){
        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }

    public void startBackward(double speed){
        FR.setPower(-speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(-speed);
    }

    public void startRight(double speed){
        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(-speed);
    }

    public void startLeft(double speed){
        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(speed);
    }

    public void stopDrive(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void rightTurn(double angle, double speed){
        double target = gyro.getIntegratedZValue() - angle;
        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(-speed);
        BL.setPower(speed);
        while(opModeIsActive() && gyro.getIntegratedZValue() > target){
            idle();
        }
        stopDrive();
    }

    public void leftTurn(double angle, double speed){
        double target = gyro.getIntegratedZValue() + angle;
        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(speed);
        BL.setPower(-speed);
        while(opModeIsActive() && gyro.getIntegratedZValue() < target){
            idle();
        }
        stopDrive();
    }


    public void moveRightInches(double inches, double speed){
        double target = -inches * strafeTicsPerInch;
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startRight(speed);
        while(opModeIsActive() && FL.getCurrentPosition() < target){
            telemetry.addData("FR",FR.getCurrentPosition());
            telemetry.addData("BR",BR.getCurrentPosition());
            telemetry.addData("FL",FL.getCurrentPosition());
            telemetry.addData("BL",BL.getCurrentPosition());
            telemetry.update();
            idle();
        }
        stopDrive();
    }

    public void moveLeftInches(double inches, double speed){
        double target = inches * strafeTicsPerInch;
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startLeft(speed);
        while(opModeIsActive() && FL.getCurrentPosition() > target){
            idle();
        }
        stopDrive();
    }

    public void moveForwardInches(double inches, double speed){
        double target = inches * ticsPerInch;
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startForward(speed);
        while(opModeIsActive() && FL.getCurrentPosition() < target){
            idle();
        }
        stopDrive();
    }

    public void moveBackwardInches(double inches, double speed){
        double target = inches*ticsPerInch;
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startBackward(speed);
        while(opModeIsActive() && FL.getCurrentPosition() > -target){
            idle();
        }
        stopDrive();
    }

}