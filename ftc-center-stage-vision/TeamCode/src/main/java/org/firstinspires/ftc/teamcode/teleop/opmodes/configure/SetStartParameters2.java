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

    private static final int MAX_DELAY = 10;
    private int currentStartDelay;
    private FTCButton increaseStartDelay;
    private FTCButton decreaseStartDelay;

    EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions;
    private boolean endPositionsChanged = false;
    private FTCButton endPositionLeft;
    private FTCButton endPositionRight;
    private FTCButton factoryReset; // GAMEPAD_1_START

    // DPAD left toggle button position A (default) is the standard button layout.
    // DPAD left Toggle button position B is the Qualia layout.
    private enum Mode {STANDARD, QUALIA}

    private Mode mode = Mode.STANDARD;
    private FTCToggleButton toggleMode;

    private FTCButton modalButton1A; // standard opModeBlueA2, Qualia wall_truss
    private FTCButton modalButton1X; // standard opModeBlueA4, QWualia center_truss path
    private FTCButton modalButton1Y; // standard opModeRedF4, Qualia stage_door path
    private FTCButton modalButton1B; // standard opModeRedF2

    // Qualia only
    private FTCToggleButton midPointDelay;

    @Override
    public void runOpMode() {
        increaseStartDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_UP);
        decreaseStartDelay = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_DOWN);

        endPositionLeft = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_LEFT);
        endPositionRight = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_DPAD_RIGHT);
        factoryReset = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_START);

        // Toggle button for switching between standard mode and Qualia mode.
        // DPAD left toggle button position A (default) is the standard button layout.
        // DPAD left Toggle button position B is the Qualia layout.
        toggleMode = new FTCToggleButton(this, FTCButton.ButtonValue.GAMEPAD_1_LEFT_BUMPER);

        // The meaning of these buttons depends upon the current mode.
        modalButton1A = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_A);
        modalButton1X = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_X);
        modalButton1Y = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_Y);
        modalButton1B = new FTCButton(this, FTCButton.ButtonValue.GAMEPAD_1_B);

        // Qualia only
        // Toggle button for switching between a post_spike delay and a pre_backstage delay.
        // DPAD right toggle button position A (default) is for the post_spike delay.
        // DPAD right Toggle button position B is for the pre_backstage delay.
        midPointDelay = new FTCToggleButton(this, FTCButton.ButtonValue.GAMEPAD_1_RIGHT_BUMPER);

        String fullXMLDir = WorkingDirectory.getWorkingDirectory() + RobotConstants.XML_DIR;
        try {
            startParametersXML = new StartParametersXML(fullXMLDir);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new RuntimeException(e);
        }

        //## Note: StartParameters.QualiaStartParameters may be null because
        // these parameters are optional. If this object is null then we will
        // disable the toggle button and all of the Qualia actions.
        startParameters = startParametersXML.getStartParameters();

        int startDelay;
        currentStartDelay = startDelay = startParameters.autoStartDelay;

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

        autoEndingPositions = new EnumMap<>(startParameters.autoEndingPositions); // copy

        while (!isStarted() && !isStopRequested()) {
            updateButtons();
            updatePlayer1();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        } // while

        if (opModeIsActive()) {
            //**TODO Detect Qualia changes also ...
            if (startDelay != currentStartDelay || endPositionsChanged) {
                if (startDelay != currentStartDelay) {
                    startParametersXML.setAutoStartDelay(currentStartDelay);
                    RobotLog.ii(TAG, "Changed start delay to " + currentStartDelay);
                    telemetry.addLine("Changed start delay to " + currentStartDelay);
                }

                if (endPositionsChanged) {
                    RobotLog.ii(TAG, "Changed one or more ending positions");
                    telemetry.addLine("Changed one or more ending positions");
                }

                startParametersXML.writeStartParametersFile();
                RobotLog.ii(TAG, "Writing StartParameters.xml");
                telemetry.addLine("Writing StartParameters.xml");
                telemetry.update();
                sleep(1500);
            }
        }
    }

    private void updateButtons() {
        increaseStartDelay.update();
        decreaseStartDelay.update();
        endPositionLeft.update();
        endPositionRight.update();
        factoryReset.update();

        toggleMode.update();
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
        } else {
            //**TODO call Qualia update methods
        }
    }

    private void updateMode() {
        if (toggleMode.is(FTCButton.State.TAP)) {
            if (toggleMode.toggle() == FTCToggleButton.ToggleState.A)
                mode = Mode.STANDARD;
            else {
                if (startParameters.qualiaStartParameters == null) {
                    //**TODO telemetry
                    return;
                }
                mode = Mode.QUALIA;
            }
        }
    }

    private void updateIncreaseStartDelay() {
        if (increaseStartDelay.is((FTCButton.State.TAP))) {
            currentStartDelay = currentStartDelay < MAX_DELAY ? ++currentStartDelay : MAX_DELAY;
            if (currentStartDelay == MAX_DELAY)
                telemetry.addLine("Start delay is at the maximum of " + MAX_DELAY);
            else
                telemetry.addLine("Start delay increased to " + currentStartDelay);
            telemetry.update();
        }
    }

    private void updateDecreaseStartDelay() {
        if (decreaseStartDelay.is((FTCButton.State.TAP))) {
            currentStartDelay = currentStartDelay >= 1 ? --currentStartDelay : 0; // 0 or positive
            if (currentStartDelay == 0)
                telemetry.addLine("Start delay is at the minimum of 0");
            else
                telemetry.addLine("Start delay decreased to " + currentStartDelay);
            telemetry.update();
        }
    }

    private void updateOpModeBlueA2() {
        setEndPosition(RobotConstantsCenterStage.OpMode.BLUE_A2, modalButton1A, endPositionLeft, endPositionRight);
    }

    private void updateOpModeBlueA4() {
        setEndPosition(RobotConstantsCenterStage.OpMode.BLUE_A4, modalButton1X, endPositionLeft, endPositionRight);
    }

    private void updateOpModeRedF2() {
        setEndPosition(RobotConstantsCenterStage.OpMode.RED_F2, modalButton1B, endPositionLeft, endPositionRight);
    }

    private void updateOpModeRedF4() {
        setEndPosition(RobotConstantsCenterStage.OpMode.RED_F4, modalButton1Y, endPositionLeft, endPositionRight);
    }

    private void setEndPosition(RobotConstantsCenterStage.OpMode pOpMode, FTCButton pOpModeButton,
                                FTCButton pEndPositionLeftButton, FTCButton pEndPositionRightButton) {
        if (pOpModeButton.is(FTCButton.State.TAP) || pOpModeButton.is(FTCButton.State.HELD)) {
            if (pOpModeButton.is(FTCButton.State.TAP)) {
                // Display the current end position.
                RobotConstantsCenterStage.AutoEndingPosition endPosition = autoEndingPositions.get(pOpMode);
                telemetry.addLine(pOpMode + " end position: " + endPosition);
                telemetry.update();
                return;
            }

            // Button is held - check for a LEFT or RIGHT selection from the DPAD.
            RobotConstantsCenterStage.AutoEndingPosition endPosition = autoEndingPositions.get(pOpMode);
            if (pEndPositionLeftButton.is(FTCButton.State.TAP)) {
                if (endPosition == RobotConstantsCenterStage.AutoEndingPosition.LEFT) {
                    telemetry.addLine("Ending position is already set to LEFT for " + pOpMode);
                    telemetry.update();
                    return;
                }

                endPositionsChanged = true;
                startParametersXML.setAutoEndingPosition(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.LEFT.toString());
                autoEndingPositions.put(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.LEFT);

                telemetry.addLine("Ending position set to LEFT for " + pOpMode);
                telemetry.update();
            } else if (pEndPositionRightButton.is(FTCButton.State.TAP)) {
                if (endPosition == RobotConstantsCenterStage.AutoEndingPosition.RIGHT) {

                    telemetry.addLine("Ending position is already set to RIGHT for " + pOpMode);
                    telemetry.update();
                    return;
                }

                endPositionsChanged = true;
                startParametersXML.setAutoEndingPosition(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.RIGHT.toString());
                autoEndingPositions.put(pOpMode, RobotConstantsCenterStage.AutoEndingPosition.RIGHT);
                telemetry.addLine("Ending position set to RIGHT for " + pOpMode);
                telemetry.update();
            }
        }
    }
}