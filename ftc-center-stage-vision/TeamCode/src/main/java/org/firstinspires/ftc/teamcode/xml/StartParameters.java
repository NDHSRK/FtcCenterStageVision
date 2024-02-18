package org.firstinspires.ftc.teamcode.xml;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.teamcode.common.RobotConstantsCenterStage;

import java.util.EnumMap;

// Starting parameters that can be set via a dedicated TeleOp
// OpMode and read by either an Autonomous or TeleOp competition
// OpMode.
public class StartParameters {

    public final String robotConfigFilename;
    public final String robotActionFilename;
    public final int autoStartDelay;
    public final EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> autoEndingPositions;

    public final QualiaStartParameters qualiaStartParameters;
    public StartParameters(String pRobotConfigFilename, String pRobotActionFilename, int pAutoStartDelay,
                           EnumMap<RobotConstantsCenterStage.OpMode, RobotConstantsCenterStage.AutoEndingPosition> pAutoEndingPositions,
                           QualiaStartParameters pQualiaStartParameters) {
        robotConfigFilename = pRobotConfigFilename;
        robotActionFilename = pRobotActionFilename;
        autoStartDelay = pAutoStartDelay;
        autoEndingPositions = pAutoEndingPositions;
        qualiaStartParameters = pQualiaStartParameters;
    }

    public static class QualiaStartParameters {
        public enum Path {STAGE_DOOR, CENTER_TRUSS, WALL_TRUSS}
        public enum MidPathDelayPoint {POST_SPIKE, PRE_BACKSTAGE}

        public final Path path;
        public final int midPathDelayPostSpike;
        public final int midPathDelayPreBackstage;

        public QualiaStartParameters(Path pPath, int pMidPathDelayPostSpike, int pMidPathDelayPreBackstage) {
            path = pPath;
            midPathDelayPostSpike = pMidPathDelayPostSpike;
            midPathDelayPreBackstage = pMidPathDelayPreBackstage;
        }
    }

}