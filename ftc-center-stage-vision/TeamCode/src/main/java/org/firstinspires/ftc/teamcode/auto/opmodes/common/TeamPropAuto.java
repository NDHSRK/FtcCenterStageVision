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

package org.firstinspires.ftc.teamcode.auto.opmodes.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.BackdropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropReturn;
import org.firstinspires.ftc.teamcode.auto.xml.BackdropParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.TeamPropParametersXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.common.xml.SpikeWindowMappingXML;
import org.firstinspires.ftc.teamcode.robot.FTCRobotConfigVision;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameAccess;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.Objects;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;

public class TeamPropAuto {
    private static final String TAG = TeamPropAuto.class.getSimpleName();

    private final LinearOpMode linear;
    private final RobotConstants.Alliance alliance;
    private final FTCRobotConfigVision robot;
    private final int autoStartDelay;
    private final EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions;
    TeamPropParameters teamPropParameters;
    SpikeWindowMapping opModeSpikeWindowMapping;
    private final BackdropParameters backdropParameters;
    private final EnumSet<RobotConstantsCenterStage.InternalWebcamId> openWebcams = EnumSet.noneOf(RobotConstantsCenterStage.InternalWebcamId.class);

    RobotConstantsCenterStage.InternalWebcamId openWebcam = RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS;

    public TeamPropAuto(LinearOpMode pLinear, RobotConstants.Alliance pAlliance,
                        RobotConstantsCenterStage.OpMode pOpMode)
            throws ParserConfigurationException, SAXException, XPathException, IOException {
        linear = pLinear;
        alliance = pAlliance;

        // Get the camera configuration from RobotConfig.xml.
        robot = new FTCRobotConfigVision(pLinear, RobotConstants.RunType.AUTONOMOUS);

        // Prepare for image recognition.
        // Get the directory for the various configuration files.
        String workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Get the configurable delay at Autonomous startup.
        autoStartDelay = robot.startParameters.autoStartDelay;

        // Get the ending positions of the robot for all competition
        // OpModes.
        autoEndingPositions = robot.startParameters.autoEndingPositions;

        // Read the parameters for team prop recognition from the xml file.
        TeamPropParametersXML teamPropParametersXML = new TeamPropParametersXML(xmlDirectory);
        teamPropParameters = teamPropParametersXML.getTeamPropParameters();
        RobotLog.ii(TAG, "Done parsing TeamPropParameters.xml");

        // Note: if no COMPETITION or AUTO_TEST OpMode in RobotAction.XML contains
        // the action FIND_TEAM_PROP then collectedSpikeWindowData will be empty.
        SpikeWindowMappingXML spikeWindowMappingXML = new SpikeWindowMappingXML(robot.startParameters.robotActionFilename);
        opModeSpikeWindowMapping = spikeWindowMappingXML.collectSpikeWindowMapping(pOpMode);

        if (opModeSpikeWindowMapping == null)
            throw new AutonomousRobotException(TAG, "Element 'FIND_TEAM_PROP' not found under OpMode " + pOpMode);

        // Read the parameters for the backdrop from the xml file.
        BackdropParametersXML backdropParametersXML = new BackdropParametersXML(xmlDirectory);
        backdropParameters = backdropParametersXML.getBackdropParameters();

        RobotLog.ii(TAG, "Done parsing BackdropParameters.xml");

        // Start the front webcam with the raw webcam frame processor.
        // We can start a camera by using the <START_CAMERA> action in RobotAction.xml
        // but since the first task in Autonomous is to find the Team Prop, we save
        // time by starting the front webcam here with two processors: one for raw
        // frames (enabled) and one for AprilTags (disabled) OR a single processor for
        // raw frames (enabled). The only time this camera might not be in the
        // configuration is during testing.
        if (robot.configuredWebcams != null) { // if webcam(s) are configured in
            VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                    robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM);
            if (frontWebcamConfiguration != null) {
                // The front webcam may configured for raw frames only or for both
                // raw frames and AprilTags.
                EnumMap<RobotConstantsCenterStage.ProcessorIdentifier, Pair<VisionProcessor, Boolean>> assignedProcessors =
                        new EnumMap<>(RobotConstantsCenterStage.ProcessorIdentifier.class);
                VisionProcessor rawFrameProcessor = new RawFrameProcessor.Builder().build();
                assignedProcessors.put(RobotConstantsCenterStage.ProcessorIdentifier.RAW_FRAME, Pair.create(rawFrameProcessor, true));

                if (frontWebcamConfiguration.processorIdentifiers.contains(RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG)) {
                    VisionProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                            // Follow the MultiPortal sample, which only includes setLensIntrinsics
                            .setLensIntrinsics(frontWebcamConfiguration.cameraCalibration.focalLengthX,
                                    frontWebcamConfiguration.cameraCalibration.focalLengthY,
                                    frontWebcamConfiguration.cameraCalibration.principalPointX,
                                    frontWebcamConfiguration.cameraCalibration.principalPointY)
                            .build();

                    assignedProcessors.put(RobotConstantsCenterStage.ProcessorIdentifier.APRIL_TAG, Pair.create(aprilTagProcessor, false));
                }

                VisionPortalWebcam visionPortalWebcam = new VisionPortalWebcam(frontWebcamConfiguration, assignedProcessors);
                frontWebcamConfiguration.setVisionPortalWebcam(visionPortalWebcam);
                if (!visionPortalWebcam.waitForWebcamStart(2000))
                    throw new AutonomousRobotException(TAG, "Unable to start front webcam");

                openWebcams.add(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM);
            }
        }
    }

    public RobotConstantsCenterStage.TeamPropLocation
    runTeamPropRecognition() throws InterruptedException {

        // Find the location of the Team Prop.
        String webcamIdString = opModeSpikeWindowMapping.imageParameters.image_source.toUpperCase();
        RobotConstantsCenterStage.InternalWebcamId webcamId =
                RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
        VisionPortalWebcamConfiguration.ConfiguredWebcam webcam =
                Objects.requireNonNull(robot.configuredWebcams.get(webcamId), TAG + " Webcam " + webcamId + " is not in the current configuration");

        if (!openWebcams.contains(webcamId))
            throw new AutonomousRobotException(TAG, "Attempt to find the team prop using webcam " + webcamId + " but it is not open");

        Pair<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> rawFrameProcessor =
                webcam.getVisionPortalWebcam().getActiveProcessor();
        if (rawFrameProcessor.first != RobotConstantsCenterStage.ProcessorIdentifier.RAW_FRAME)
            throw new AutonomousRobotException(TAG, "The active processor is not RAW_FRAME");

        RawFrameAccess rawFrameAccess = new RawFrameAccess((RawFrameProcessor) rawFrameProcessor.second);

        // Get the recognition path from the XML file.
        RobotConstantsCenterStage.TeamPropRecognitionPath teamPropRecognitionPath =
                opModeSpikeWindowMapping.recognitionPath;
        RobotLog.dd(TAG, "Recognition path " + teamPropRecognitionPath);

        // Perform image recognition.
        TeamPropRecognition teamPropRecognition = new TeamPropRecognition(alliance);
        TeamPropReturn teamPropReturn = teamPropRecognition.recognizeTeamProp(rawFrameAccess, teamPropRecognitionPath, teamPropParameters, opModeSpikeWindowMapping);
        RobotConstantsCenterStage.TeamPropLocation finalTeamPropLocation;
        if (teamPropReturn.recognitionResults == RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR ||
                teamPropReturn.recognitionResults == RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL) {
            // Something went wrong during recognition but don't crash; use the default location of CENTER_SPIKE.
            finalTeamPropLocation = RobotConstantsCenterStage.TeamPropLocation.CENTER_SPIKE;
            linear.telemetry.addLine("Error in computer vision subsystem; using default location");
            linear.telemetry.update();
            RobotLog.dd(TAG, "Error in computer vision subsystem; using default location of CENTER_SPIKE");
        } else {
            finalTeamPropLocation = teamPropReturn.teamPropLocation;
        }

        return finalTeamPropLocation;
    }

}
