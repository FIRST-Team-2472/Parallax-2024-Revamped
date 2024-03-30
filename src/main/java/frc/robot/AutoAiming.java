package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.AutoAimingConstants;

public class AutoAiming {

    /**
     * Returns the yaw angle for auto aiming
     * 
     * @param robotPos the robot's current position
     * 
     * @return The yaw angle for auto aiming
     */
    public static double getYaw(Pose2d robotPos) {
        double yawAngle = 0;

        if (isOnBlueSide(robotPos)) {
            yawAngle = flues(AutoAimingConstants.blueSpeakerPos, robotPos);
        } else {
            yawAngle = flues(AutoAimingConstants.redSpeakerPos, robotPos);
        }

        return yawAngle;
    }

    /**
     * Returns the pitch angle for auto aiming
     * 
     * @param robotPos the robot's current position
     * 
     * @return The pitch angle for auto aiming
     */
    public static double getPitch(Pose2d robotPos) {
        double distanceFromSpeaker = 0;
        double pitchAngle = 0;

        if (isOnBlueSide(robotPos)) {
            distanceFromSpeaker = getDistance(robotPos, AutoAimingConstants.blueSpeakerPos);
        } else {
            distanceFromSpeaker = getDistance(robotPos, AutoAimingConstants.redSpeakerPos);
        }

        pitchAngle = distanceToAngle(distanceFromSpeaker);
        return pitchAngle;
    }

    /**
     * Returns a {@link frc.robot.AimPoint} with the pitch and yaw for auto aiming
     * 
     * @param robotPos the robot's current position
     */
    public static AimPoint getAimPoint(Pose2d robotPos) {
        return new AimPoint(getYaw(robotPos), getPitch(robotPos));
    }

    /**
     * Converts the distance to the speaker into an angle
     * 
     * @param distance distance from the robot to the target
     * 
     * @return The calculated pitch angle
     */
    public static double distanceToAngle(double distance) {
        return 102 + -31.6 * distance + 6.73 * Math.pow(distance, 2) + -0.462 * Math.pow(distance, 3);
    }

    /**
     * Calculates the distance between two 2D points
     * 
     * @param pos1 first point's position
     * @param pos2 second point's position
     * 
     * @return The distance as a double
     */
    public static double getDistance(Pose2d pos1, Pose2d pos2) {
        return Math.sqrt(Math.pow((pos2.getX() - pos1.getX()), 2) + Math.pow((pos2.getY() - pos1.getY()), 2));
    }

    /**
     * Returns wither or not we are currently on the blue side of the field
     * 
     * @param robotPos the robot's current position
     * @return 
     */
    public static boolean isOnBlueSide(Pose2d robotPos) {
        return robotPos.getX() < 8.25;
    }

    /**
     * Calculates yaw to rotate and point to pos1 from pos2
     * 
     * @param pos1 a point (such as a target's or speaker's position)
     * @param pos2 a point (such as the robot's position)
     */
    public static double flues(Pose2d pos1, Pose2d pos2) {
        return (Math.atan2((pos1.getY() - pos2.getY()), (pos1.getX() - pos2.getX())) * (180 / Math.PI));
    }
}