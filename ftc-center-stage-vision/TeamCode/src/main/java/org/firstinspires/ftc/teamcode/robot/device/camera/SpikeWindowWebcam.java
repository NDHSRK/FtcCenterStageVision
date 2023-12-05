package org.firstinspires.ftc.teamcode.robot.device.camera;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;
import org.firstinspires.ftc.vision.VisionProcessor;

public class SpikeWindowWebcam extends VisionPortalWebcam {
    private static final String TAG = SpikeWindowWebcam.class.getSimpleName();

    public SpikeWindowWebcam(VisionPortalWebcamConfiguration.ConfiguredWebcam pConfiguredWebcam,
                             Pair<RobotConstantsCenterStage.ProcessorIdentifier, VisionProcessor> pAssignedProcessor) {
        super(pConfiguredWebcam, pAssignedProcessor);

        if (activeProcessorId != RobotConstantsCenterStage.ProcessorIdentifier.SPIKE_WINDOW)
            throw new AutonomousRobotException(TAG, "SPIKE_WINDOW is not the active processor");
    }

}
