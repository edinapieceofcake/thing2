/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.edinaftc.opmodes.vision;

import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.vision.VuforiaCamera;
import com.edinaftc.relicrecovery.vision.DynamicJewelTracker;
import com.edinaftc.relicrecovery.vision.RelicRecoveryVuMarkTracker;
import com.edinaftc.skystone.vision.SkyStoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class SkyStone extends OpMode {
    private VuforiaCamera camera;
    private SkyStoneDetector skyStoneDetector;
    private Stickygamepad _gamepad1;
    private Location location = Location.left;
    private enum Location {
        left, middle, right, line
    }

    @Override
    public void init() {
        skyStoneDetector = new SkyStoneDetector();
        camera = new VuforiaCamera();
        _gamepad1 = new Stickygamepad(gamepad1);
        camera.addTracker(skyStoneDetector);
        camera.initialize();
    }

    @Override
    public void loop() {
        _gamepad1.update();
        if (_gamepad1.x) {
            if (location == Location.line) {
                skyStoneDetector.lineX -= 10;
            } else if (location == Location.left) {
                skyStoneDetector.cx0 += 10;
            } else if (location == Location.middle) {
                skyStoneDetector.cx1 += 10;
            } else if (location == Location.right){
                skyStoneDetector.cx2 += 10;
            }
        }

        if (_gamepad1.y) {
            if (location == Location.left) {
                skyStoneDetector.cx0 -= 10;
            } else if (location == Location.middle) {
                skyStoneDetector.cx1 -= 10;
            } else if (location == Location.right){
                skyStoneDetector.cx2 -= 10;
            }
        }

        if (_gamepad1.b) {
            if (location == Location.line) {
                skyStoneDetector.lineX += 10;
            } else if (location == Location.left) {
                skyStoneDetector.cy0 += 10;
            } else if (location == Location.middle) {
                skyStoneDetector.cy1 += 10;
            } else if (location == Location.right) {
                skyStoneDetector.cy2 += 10;
            }
        }

        if (_gamepad1.a) {
            if (location == Location.left) {
                skyStoneDetector.cy0 -= 10;
            } else if (location == Location.middle) {
                skyStoneDetector.cy1 -= 10;
            } else if (location == Location.right){
                skyStoneDetector.cy2 -= 10;
            }
        }

        if (_gamepad1.left_bumper) {
            if (location == Location.middle) {
                location = Location.middle.left;
            } else if (location == Location.right) {
                location = Location.middle.middle;
            } else if (location == Location.left){
                location = Location.line;
            } else {
                location = Location.line;
            }
        }

        if (_gamepad1.right_bumper) {
            if (location == Location.line) {
                location = Location.left;
            } else if (location == Location.left) {
                location = Location.middle;
            } else if (location == Location.middle) {
                location = Location.right;
            } else {
                location = Location.right;
            }
        }

        telemetry.addData("left (x, y)", "%f %f", skyStoneDetector.cx0, skyStoneDetector.cy0);
        telemetry.addData("middle (x, y)", "%f %f", skyStoneDetector.cx1, skyStoneDetector.cy1);
        telemetry.addData("right (x, y)", "%f %f", skyStoneDetector.cx2, skyStoneDetector.cy2);
        telemetry.addData("line x", "%f", skyStoneDetector.lineX);
        telemetry.addData("dot location", location);
        telemetry.addData("location ", skyStoneDetector.getLocation());
        telemetry.update();
    }

    @Override
    public void stop() {
        camera.close();
    }
}