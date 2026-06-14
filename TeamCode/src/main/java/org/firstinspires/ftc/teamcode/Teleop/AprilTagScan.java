package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.auto.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d   ;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.util.List;

public class AprilTagScan implements Action {

    private final Action turnAction;
    private final AprilTagProcessor aprilTag;
    private final int targetTagId;

    private boolean tagFound = false;
    private AprilTagDetection foundDetection = null;

    /**
     * @param drive       your MecanumDrive (or similar)
     * @param angle       angle to turn, in radians (matches TurnAction signature)
     * @param aprilTag    the AprilTagProcessor instance (already attached to a VisionPortal)
     * @param targetTagId the tag ID that, if seen, stops the turn early
     */
    public AprilTagScan(MecanumDrive drive, double angle, AprilTagProcessor aprilTag, int targetTagId) {
        this.turnAction = drive.actionBuilder(drive.localizer.getPose())
                .turn(angle)
                .build();
        this.aprilTag = aprilTag;
        this.targetTagId = targetTagId;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        // 1. Check for the target tag first
        List<AprilTagDetection> detections = aprilTag.getDetections();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (detection.id == targetTagId) {
                    tagFound = true;
                    foundDetection = detection;

                    packet.put("Position x", detection.robotPose.getPosition().x);
                    packet.put("Position y", detection.robotPose.getPosition().y);
                    packet.put("Yaw", detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
                    return false; // stop the action — tag found, turn ends here
                }
            }
        }


        // 2. No target tag yet — continue the turn
        packet.put("Tags visible", detections.size());
        boolean stillTurning = turnAction.run(packet);

        // If the turn finishes naturally without finding the tag, action ends
        return stillTurning;
    }

    public boolean isTagFound() {
        return tagFound;
    }

    public AprilTagDetection getFoundDetection() {
        return foundDetection;
    }
}