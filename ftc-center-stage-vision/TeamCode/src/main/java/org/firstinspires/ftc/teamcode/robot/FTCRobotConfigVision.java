package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.android.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.teamcode.common.StartParameters;
import org.firstinspires.ftc.teamcode.common.xml.StartParametersXML;
import org.firstinspires.ftc.teamcode.robot.device.camera.VisionPortalWebcamConfiguration;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.EnumMap;
import java.util.Optional;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class FTCRobotConfigVision {

    private static final String TAG = FTCRobotConfigVision.class.getSimpleName();

    private final HardwareMap hardwareMap;
    public final StartParameters startParameters;

    public final EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcams;

    public FTCRobotConfigVision(LinearOpMode pLinearOpMode, RobotConstants.RunType pRunType) {
        hardwareMap = pLinearOpMode.hardwareMap;

        RobotLog.ii(TAG, TAG + " constructor");

        String workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.XML_DIR;

        // Get the hardware configuration parameters from RobotConfig.xml.
        try {
            // Get the startup parameters (including the exact file name of
            // RobotConfig XXX.xml).
            // Get the configurable startup parameters.
            StartParametersXML startParametersXML = new StartParametersXML(xmlDirectory);
            startParameters = startParametersXML.getStartParameters();
            RobotLog.ii(TAG, "Configuring the robot from " + startParameters.robotConfigFilename);

            RobotConfigXML configXML = new RobotConfigXML(startParameters.robotConfigFilename);
            XPathAccess configXPath;

            // In a competition the webcam(s) would be configured in and
            // used in Autonomous but not in TeleOp so we can't just check
            // the configuration file.
            if (pRunType == RobotConstants.RunType.TELEOP) {
                configuredWebcams = null;
            } else {
                // Any configured VisionPortal webcams?
                EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcamsLocal;
                configXPath = configXML.getPath("VISION_PORTAL_WEBCAM");
                String webcamYesNo = configXPath.getRequiredTextInRange("@configured", configXPath.validRange("yes", "no"));
                RobotLog.ii(TAG, "VisionPortal webcam configuration option: " + webcamYesNo);

                if (webcamYesNo.equals("yes")) {
                    configuredWebcamsLocal = configXML.getConfiguredWebcams();
                    RobotLog.ii(TAG, "Number of webcams configured " + configuredWebcamsLocal.size());
                    if (configuredWebcamsLocal.size() > 2)
                        throw new AutonomousRobotException(TAG, "CenterStage season: only two webcams at mnost are supported");

                    matchHardwareWebcamsWithConfiguredWebcams(configuredWebcamsLocal);
                } else
                    configuredWebcamsLocal = null;

                configuredWebcams = configuredWebcamsLocal; // needed to preserve "final"
            }
        } catch (ParserConfigurationException | SAXException | XPathExpressionException |
                 IOException ex) {
            String eMessage = ex.getMessage() == null ? "**no error message**" : ex.getMessage();
            throw new AutonomousRobotException(TAG, "IOException " + eMessage);
        }
    }

    // The FTC SDK uses a string such as "Webcam 1" to connect
    // a webcam via the HardwareMap and to create a WebcamName
    // object. Use the webcam's serial number in its WebcamName
    // object to associate the webcam with its counterpart in
    // RobotConfig.xml.
    private void matchHardwareWebcamsWithConfiguredWebcams(EnumMap<RobotConstantsCenterStage.InternalWebcamId, VisionPortalWebcamConfiguration.ConfiguredWebcam> pConfiguredWebcams) {
        String webcamId;
        for (int i = 1; i <= pConfiguredWebcams.size(); i++) {
            webcamId = "Webcam " + new DecimalFormat("0").format(i);
            WebcamName webcamName = hardwareMap.get(WebcamName.class, webcamId);
            if (!webcamName.isWebcam() || !webcamName.isAttached())
                throw new AutonomousRobotException(TAG, "Webcam " + webcamId +
                        " is not a webcam or is not attached");

            Optional<VisionPortalWebcamConfiguration.ConfiguredWebcam> configuredWebcam = pConfiguredWebcams.values().stream()
                    .filter(webcam -> webcam.serialNumber.equals(webcamName.getSerialNumber().toString()))
                    .findFirst();

            if (!configuredWebcam.isPresent())
                throw new AutonomousRobotException(TAG,
                        "No configured webcam for serial number: " + webcamName.getSerialNumber());

            // Now that we have the correct association, add the WebcamName to the
            // configured camera.
            VisionPortalWebcamConfiguration.ConfiguredWebcam configuredWebcamObject = configuredWebcam.get();
            configuredWebcamObject.setWebcamName(webcamName);
            RobotLog.ii(TAG, "Webcam hardware device " + webcamId +
                    " is associated by serial number " + webcamName.getSerialNumber() +
                    " with configured webcam " + configuredWebcamObject.internalWebcamId);
        }
    }

}

