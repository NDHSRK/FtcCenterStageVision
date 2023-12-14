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

package org.firstinspires.ftc.teamcode.auto.opmodes.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.SpikeWindowMapping;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.TeamPropReturn;
import org.firstinspires.ftc.teamcode.common.xml.SpikeWindowMappingXML;
import org.firstinspires.ftc.teamcode.auto.xml.TeamPropParametersXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.robot.FTCRobotConfigVision;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameProcessor;
import org.firstinspires.ftc.teamcode.robot.device.camera.RawFrameWebcam;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.Objects;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class TeamPropAuto {
    private static final String TAG = TeamPropAuto.class.getSimpleName();

    private final LinearOpMode linear;
    private final RobotConstants.Alliance alliance;
    private final FTCRobotConfigVision robot;
    TeamPropParameters teamPropParameters;
    SpikeWindowMapping opModeSpikeWindowMapping;
    RobotConstantsCenterStage.InternalWebcamId openWebcam = RobotConstantsCenterStage.InternalWebcamId.WEBCAM_NPOS;

    public TeamPropAuto(LinearOpMode pLinear, RobotConstants.Alliance pAlliance,
                        RobotConstantsCenterStage.OpMode pOpMode) throws InterruptedException {
        linear = pLinear;
        alliance = pAlliance;

        // Get the camera configuration from RobotConfig.xml.
        robot = new FTCRobotConfigVision(pLinear, RobotConstants.RunType.AUTONOMOUS);

        // Prepare for image recognition.
        // Get the directory for the various configuration files.
        String workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Read the parameters for team prop recognition from the xml file.
        TeamPropParametersXML teamPropParametersXML = new TeamPropParametersXML(xmlDirectory);
        try {
            teamPropParameters = teamPropParametersXML.getTeamPropParameters();
        } catch (XPathExpressionException e) {
            throw new RuntimeException(e);
        }

        // Note: if no COMPETITION or AUTO_TEST OpMode in RobotAction.XML contains
        // the action FIND_TEAM_PROP then collectedSpikeWindowData will be empty.
        SpikeWindowMappingXML spikeWindowMappingXML;
        try {
            spikeWindowMappingXML = new SpikeWindowMappingXML(xmlDirectory);
        } catch (ParserConfigurationException | SAXException | IOException e) {
            throw new RuntimeException(e);
        }

        try {
            opModeSpikeWindowMapping = spikeWindowMappingXML.collectSpikeWindowMapping(pOpMode);
        } catch (XPathExpressionException e) {
            throw new RuntimeException(e);
        }

        if (opModeSpikeWindowMapping == null)
            throw new AutonomousRobotException(TAG, "Element 'FIND_TEAM_PROP' not found under OpMode " + pOpMode);

        // Start the front webcam with the webcam frame processor.
        // We can start a camera by using the <START_CAMERA> action in RobotAction.xml
        // but since the first task in Autonomous is to find the Team Prop, we save
        // time by starting the front webcam here with a processor for raw frames. The
        // only time this camera might not be in the configuration is during testing.
        if (robot.configuredWebcams != null) { // if webcam(s) are configured in
            VisionPortalWebcamConfiguration.ConfiguredWebcam frontWebcamConfiguration =
                    robot.configuredWebcams.get(RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM);
            if (frontWebcamConfiguration != null) {
                VisionProcessor webcamFrameProcessor = new RawFrameProcessor.Builder().build();
                RawFrameWebcam rawFrameWebcam = new RawFrameWebcam(frontWebcamConfiguration,
                        Pair.create(RobotConstantsCenterStage.ProcessorIdentifier.RAW_FRAME, webcamFrameProcessor));
                if (!rawFrameWebcam.waitForWebcamStart(2000))
                    throw new AutonomousRobotException(TAG, "Unable to start front webcam");
                frontWebcamConfiguration.setVisionPortalWebcam(rawFrameWebcam);
                openWebcam = RobotConstantsCenterStage.InternalWebcamId.FRONT_WEBCAM;
            }
        }
    }

    public RobotConstantsCenterStage.TeamPropLocation
    runTeamPropRecognition() throws InterruptedException {

        // Find the location of the Team Prop.
        String webcamIdString = opModeSpikeWindowMapping.imageParameters.image_source.toUpperCase();
        RobotConstantsCenterStage.InternalWebcamId webcamId =
                RobotConstantsCenterStage.InternalWebcamId.valueOf(webcamIdString);
        if (openWebcam != webcamId)
            throw new AutonomousRobotException(TAG, "Attempt to find the team prop using webcam " + webcamId + " but it is not open");

        RawFrameWebcam rawFrameWebcam = (RawFrameWebcam) Objects.requireNonNull(robot.configuredWebcams.get(webcamId)).getVisionPortalWebcam();

        // Get the recognition path from the XML file.
        RobotConstantsCenterStage.TeamPropRecognitionPath teamPropRecognitionPath =
                opModeSpikeWindowMapping.recognitionPath;
        RobotLog.dd(TAG, "Recognition path " + teamPropRecognitionPath);

        // Perform image recognition.
        TeamPropRecognition teamPropRecognition = new TeamPropRecognition(alliance);
        TeamPropReturn teamPropReturn = teamPropRecognition.recognizeTeamProp(rawFrameWebcam, teamPropRecognitionPath, teamPropParameters, opModeSpikeWindowMapping);
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
