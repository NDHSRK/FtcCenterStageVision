package org.firstinspires.ftc.teamcode.teleop.opmodes.configure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.StartParameters;
import org.firstinspires.ftc.teamcode.common.xml.StartParametersXML;
import org.firstinspires.ftc.teamcode.teleop.common.FTCButton;
import org.firstinspires.ftc.teamcode.teleop.common.FTCToggleButton;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumMap;

import javax.xml.parsers.ParserConfigurationException;

//**TODO A version of SetStartParameters that supports parameters
// specific to the Qualia FTC Team. These features are activated
// via the use of the gamedpad 1 left bumper as a mode switch.

@TeleOp(name = "SetStartParameters2", group = "Configure")
//@Disabled
public class SetStartParameters2 extends LinearOpMode {

    private static final String TAG = SetStartParameters2.class.getSimpleName();

    private StartParametersXML startParametersXML;
    private StartParameters startParameters;
    private boolean startParametersXMLChanged = false;

    private static final int MAX_DELAY = 10; // applies to all delays

    // DPAD left toggle button position A (default) is the standard button layout.
    // DPAD left Toggle button position B is the Qualia layout.
    private enum Mode {STANDARD, QUALIA}

    private Mode mode = Mode.STANDARD;
    private FTCToggleButton toggleMode;

    // This section applies to the STANDARD model only
    private int currentStartDelay;
    RobotConstantsCenterStage.OpMode currentOpMode = RobotConstantsCenterStage.OpMode.OPMODE_NPOS;
    EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions;
    private FTCButton endPositionLeft;
    private FTCButton endPositionRight;
    private FTCButton factoryReset;
    // End STANDARD mode section

    // This section applies to the QUALIA mode only.
    private StartParameters.QualiaStartParameters.Path currentPath;
    private int currentPostSpikeDelay;
    private int currentPreBackstageDelay;
    private FTCToggleButton midPointDelay;
    // End QUALIA mode section

    // This section applies to the modal use of same buttons
    private FTCButton modalIncreaseDelay; // standard increase start delay, Qualia increase mid-point delay
    private FTCButton modalDecreaseDelay; // standard decrease start delay, Qualia decrease mid-point delay
    private FTCButton modalButton1A; // standard opModeBlueA2, Qualia wall_truss
    private FTCButton modalButton1X; // standard opModeBlueA4, Qualia center_truss path
    private FTCButton modalButton1Y; // standard opModeRedF4, Qualia stage_door path
    private FTCButton modalButton1B; // standard opModeRedF2, no Qualia use
    // End modal section

    @Override
    public void runOpMode() {
        // Toggle button for switching between standard mode and Qualia mode.
        // DPAD left toggle button position A (default) is the standard button layout.
        // DPAD left Toggle button position B is the Qualia layout.
        toggleMode = new FTCToggleButton(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);

        // Button assignments for STANDARD mode.
        endPositionLeft = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_LEFT);
        endPositionRight = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_RIGHT);
        factoryReset = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_START);

        // Button assignments for QUALIA mode.
        // Toggle button for switching between a post_spike delay and a pre_backstage delay.
        // DPAD right toggle button position A (default) is for the post_spike delay.
        // DPAD right Toggle button position B is for the pre_backstage delay.
        midPointDelay = new FTCToggleButton(this, FTCButton.ButtonValue.GAMEPAD_1_RIGHT_BUMPER);

        // Modal button assignments.
        modalIncreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        modalDecreaseDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);
        modalButton1A = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        modalButton1X = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_X);
        modalButton1Y = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        modalButton1B = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_B);

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        try {
            startParametersXML = new StartParametersXML(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new RuntimeException(e);
        }

        startParameters = startParametersXML.getStartParameters();

        currentStartDelay = startParameters.autoStartDelay;
        autoEndingPositions = new EnumMap<>(startParameters.autoEndingPositions); // copy

        //## Note: StartParameters.QualiaStartParameters may be null because
        // these parameters are optional. If this object is null then we will
        // disable the toggle button and all of the Qualia actions.
        if (startParameters.qualiaStartParameters != null) {
            currentPath = startParameters.qualiaStartParameters.path;
            currentPostSpikeDelay = startParameters.qualiaStartParameters.midPathDelayPostSpike;
            currentPreBackstageDelay = startParameters.qualiaStartParameters.midPathDelayPreBackstage;
        }

        //**TODO this should be part of the main loop ... with different
        // output depending on the mode.
        telemetry.addLine("The current mode is STANDARD");
        if (startParameters.qualiaStartParameters != null)
            telemetry.addLine("Press the DPAD left bumper to toggle to QUALIA");

        telemetry.addLine("The current start delay is " + currentStartDelay);
        telemetry.addLine("DPAD_UP to increase delay; DPAD_DOWN to decrease");
        telemetry.addLine("Hold ABXY buttons to select OpMode:");
        telemetry.addLine("  A for BLUE_A2, X for BLUE_A4");
        telemetry.addLine("  Y for RED_F4, B for RED_F2");
        telemetry.addLine("Touch DPAD left or right for ending position");
        telemetry.addLine("Touch play to SAVE changes and END the OpMode");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayer1();

            //**TODO updateTelemetry();

            sleep(50); // Don't burn CPU cycles busy-looping
        } // while

        if (opModeIsActive()) {
            if (startParametersXMLChanged) {
                startParametersXML.writeStartParametersFile();
                RobotLog.ii(TAG, "Writing StartParameters.xml");
                telemetry.addLine("Writing StartParameters.xml");
            } else
                telemetry.addLine("No changes to StartParameters.xml");

            telemetry.update();
            sleep(1500);
        }
    }

    private void updateButtons() {
        toggleMode.update();

        modalIncreaseDelay.update();
        modalDecreaseDelay.update();
        endPositionLeft.update();
        endPositionRight.update();
        factoryReset.update();

        modalButton1A.update();
        modalButton1X.update();
        modalButton1Y.update();
        modalButton1B.update();
        midPointDelay.update();
    }

    private void updatePlayer1() {
        updateMode();
        if (mode == Mode.STANDARD) {
            updateIncreaseStartDelay();
            updateDecreaseStartDelay();
            updateOpModeBlueA2();
            updateOpModeBlueA4();
            updateOpModeRedF2();
            updateOpModeRedF4();
            updateEndPositionLeft();
            updateEndPositionRight();
            updateFactoryReset();
        } else {
            //**TODO call Qualia update methods
            // updateWallTrussPath();
            // updateCenterTrussPath();
            // updateStageDoorPath();
            // updateDelayPointMode();
            // updateIncreasePostSpikeDelay();
            // updateDecreasePostSpikeDelay();
            // updateIncreasePreBackstageDelay();
            // updateDecreasePreBackstageDelay();
            /*
               private FTCButton modalButton1A; // standard opModeBlueA2, Qualia wall_truss
    private FTCButton modalButton1X; // standard opModeBlueA4, Qualia center_truss path
    private FTCButton modalButton1Y; // standard opModeRedF4, Qualia stage_door path
             */
        }
    }

    private void updateMode() {
        if (toggleMode.is(FTCButton.State.TAP)) {
            if (toggleMode.toggle() == FTCToggleButton.ToggleState.A)
                mode = Mode.STANDARD;
            else {
                if (startParameters.qualiaStartParameters == null) {
                    telemetry.addLine("Toggle disallowed: Qualia elements are not present in StartParameters.xml");
                    telemetry.update();
                } else
                    mode = Mode.QUALIA;
            }
        }
    }

    // STANDARD mode methods.
    private void updateIncreaseStartDelay() {
        if (modalIncreaseDelay.is((FTCButton.State.TAP))) {
            if (currentStartDelay == MAX_DELAY)
                telemetry.addLine("Start delay is at the maximum of " + MAX_DELAY);
            else {
                ++currentStartDelay;
                startParametersXML.setAutoStartDelay(currentStartDelay);
                startParametersXMLChanged = true;
            }

            telemetry.update();
        }
    }

    private void updateDecreaseStartDelay() {
        if (modalDecreaseDelay.is((FTCButton.State.TAP))) {
            if (currentStartDelay == 0)
                telemetry.addLine("Start delay is at the minimum of 0");
            else {
                --currentStartDelay;
                startParametersXML.setAutoStartDelay(currentStartDelay);
                startParametersXMLChanged = true;
            }

            telemetry.update();
        }
    }

    private void updateOpModeBlueA2() {
        if (modalButton1A.is(FTCButton.State.TAP))
            currentOpMode = RobotConstantsCenterStage.OpMode.BLUE_A2;
    }

    private void updateOpModeBlueA4() {
        if (modalButton1X.is(FTCButton.State.TAP))
            currentOpMode = RobotConstantsCenterStage.OpMode.BLUE_A4;
    }

    private void updateOpModeRedF2() {
        if (modalButton1B.is(FTCButton.State.TAP))
            currentOpMode = RobotConstantsCenterStage.OpMode.RED_F2;
    }

    private void updateOpModeRedF4() {
        if (modalButton1Y.is(FTCButton.State.TAP))
            currentOpMode = RobotConstantsCenterStage.OpMode.RED_F4;
    }

    private void updateEndPositionLeft() {
        if (currentOpMode == RobotConstantsCenterStage.OpMode.OPMODE_NPOS) {
            telemetry.addLine("End position cannot be set: first choose an OpMode");
            telemetry.update();
            return;
        }

        if (endPositionLeft.is(FTCButton.State.TAP)) {
            RobotConstantsCenterStage.AutoEndingPosition endPosition = autoEndingPositions.get(currentOpMode);
            if (endPosition == RobotConstantsCenterStage.AutoEndingPosition.LEFT) {
                telemetry.addLine("Ending position is already set to LEFT for " + currentOpMode);
                telemetry.update();
                return;
            }

            startParametersXMLChanged = true;
            startParametersXML.setAutoEndingPosition(currentOpMode, RobotConstantsCenterStage.AutoEndingPosition.LEFT.toString());
            autoEndingPositions.put(currentOpMode, RobotConstantsCenterStage.AutoEndingPosition.LEFT);

            telemetry.addLine("Ending position set to LEFT for " + currentOpMode);
            telemetry.update();
        }
    }

    private void updateEndPositionRight() {
        if (currentOpMode == RobotConstantsCenterStage.OpMode.OPMODE_NPOS) {
            telemetry.addLine("End position cannot be set: first choose an OpMode");
            telemetry.update();
            return;
        }

        if (endPositionRight.is(FTCButton.State.TAP)) {
            RobotConstantsCenterStage.AutoEndingPosition endPosition = autoEndingPositions.get(currentOpMode);
            if (endPosition == RobotConstantsCenterStage.AutoEndingPosition.RIGHT) {
                telemetry.addLine("Ending position is already set to RIGHT for " + currentOpMode);
                telemetry.update();
                return;
            }

            startParametersXMLChanged = true;
            startParametersXML.setAutoEndingPosition(currentOpMode, RobotConstantsCenterStage.AutoEndingPosition.RIGHT.toString());
            autoEndingPositions.put(currentOpMode, RobotConstantsCenterStage.AutoEndingPosition.RIGHT);

            telemetry.addLine("Ending position set to RIGHT for " + currentOpMode);
            telemetry.update();
        }
    }

    private void updateFactoryReset() {
        if (factoryReset.is(FTCButton.State.TAP)) {
            //**TODO re-read the XML file and reset all fields and flags.
        }
    };

    // QUALIA mode methods.

    //                private FTCButton modalButton1A; // standard opModeBlueA2, Qualia wall_truss
    // private void updateWallTrussPath() {
    //        if (modalButton1A.is(FTCButton.State.TAP)) {
    //            //**TODO re-read the XML file and reset all fields and flags.
    //        }
    //    };

    //     private FTCButton modalButton1X; // standard opModeBlueA4, Qualia center_truss path
    // private void updateCenterTrussPath() {
    //        if (modalButton1X.is(FTCButton.State.TAP)) {
    //            //**TODO re-read the XML file and reset all fields and flags.
    //        }
    //    };

    //    private FTCButton modalButton1Y; // standard opModeRedF4, Qualia stage_door path
    // private void updateStageDoorPath() {
    //        if (modalButton1Y.is(FTCButton.State.TAP)) {
    //            //**TODO re-read the XML file and reset all fields and flags.
    //        }
    //    };

   /*
       private void updateDelayPointMode() { //**TODO
        if (toggleMode.is(FTCButton.State.TAP)) {
            if (toggleMode.toggle() == FTCToggleButton.ToggleState.A)
                mode = Mode.STANDARD;
            else {
                if (startParameters.qualiaStartParameters == null) {
                    telemetry.addLine("Toggle disallowed: Qualia elements are not present in StartParameters.xml");
                    telemetry.update();
                } else
                    mode = Mode.QUALIA;
            }
        }
    }
    */
    // private void updateIncreasePostSpikeDelay() {
    //        if (modalIncreaseDelay.is(FTCButton.State.TAP)) {
    //            //**TODO re-read the XML file and reset all fields and flags.
    //        }
    //    };

    // private void updateDecreasePostSpikeDelay() {
    //        if (modalDecreaseDelay.is(FTCButton.State.TAP)) {
    //            //**TODO re-read the XML file and reset all fields and flags.
    //        }
    //    };

    // private void updateIncreasePreBackstageDelay() {
    //        if (modalIncreaseDelay.is(FTCButton.State.TAP)) {
    //            //**TODO re-read the XML file and reset all fields and flags.
    //        }
    //    };

    // private void updateDecreasePreBackstageDelay() {
    //        if (modalDecreaseDelay.is(FTCButton.State.TAP)) {
    //            //**TODO re-read the XML file and reset all fields and flags.
    //        }
    //    };
}