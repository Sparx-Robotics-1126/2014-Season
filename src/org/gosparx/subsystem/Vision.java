package org.gosparx.subsystem;

import com.sun.squawk.microedition.io.FileConnection;
import com.sun.squawk.pragma.ForceInlinedPragma;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import javax.microedition.io.Connector;
import org.gosparx.IO;
import org.gosparx.util.LogWriter;

public class Vision extends GenericSubsystem {
    private LogWriter logWriter;

    private int imageLocation;//middle is 180, left is 0, right is 360
    private double imageDistance;//distance from target
    private boolean imageHotGoal;//true if hot goal in image, false if not
    private Scores scores[];

    private BinaryImage thresholdImage;
    private BinaryImage filteredImage;
    private ColorImage image;
    private static Vision vision;
    private Solenoid cameraLights;

    private TargetReport target;
    private int verticalTargets[];
    private int horizontalTargets[];
    private int verticalTargetCount, horizontalTargetCount;

    //Camera constants used for distance calculation
    private static final int Y_IMAGE_RES = 480;		//X Image resolution in pixels, should be 120, 240 or 480
    private static final double VIEW_ANGLE = 41.7;		//Axis 206 camera

    //Score limits used for target identification
    private static final int RECTANGULARITY_LIMIT = 40;
    private static final int ASPECT_RATIO_LIMIT = 55;

    //Score limits used for hot target determination
    private static final int TAPE_WIDTH_LIMIT = 50;
    private static final int VERTICAL_SCORE_LIMIT = 50;
    private static final int LR_SCORE_LIMIT = 50;

    //Minimum area of particles to be considered
    private static final int AREA_MINIMUM = 150;

    //Maximum number of particles to process
    private static final int MAX_PARTICLES = 8;

    /**
     * If true then the imaging calculations will continue If false the thread
     * will sleep saving CPU power
     */
    private boolean needImage = true;

    private AxisCamera camera;          // the axis camera object (connected to the switch)
    private CriteriaCollection cc;      // the criteria for doing the particle filter operation

    private double degrees = 0.0;
    private static final int TARGET_HEIGHT_INCHES = 32;
    public int cameraVerticalCount = 0;
    private double pixelsToInches = 0;
    private static final int CENTER_OF_CAMERA = 160;
    private double startImageTime;
    private boolean cameraResponding = false;
    private int timeOutNumber = 0;
    
    private int boundingRectHeight;
    private final static int MAX_STORED_PICTURES = 50;

    private Vision() {
        super("Vision", Thread.MIN_PRIORITY);
    }

    /**
     * starts camera and some of the image criteria
     */
    public void init() {
        logWriter = LogWriter.getInstance();
        target = new TargetReport();
        horizontalTargets = new int[MAX_PARTICLES];
        verticalTargets = new int[MAX_PARTICLES];
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        cameraLights = new Solenoid(IO.DEFAULT_SLOT, IO.CAMERA_LIGHT_RELAY);
        cameraLights.set(true);
        camera = AxisCamera.getInstance();// get an instance of the camera 
        try {
            Thread.sleep(2000);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
        while (!cameraResponding && timeOutNumber <= 30) {
            try {
                Thread.sleep(1000);
                camera.getImage();
                cameraResponding = true;
            } catch (Exception e) {
                timeOutNumber++;
            }
        }

        if (!cameraResponding) {
            log.logError("Camera not Responding");
            log.logMessage("THE CAMERA HAS FAILING (SILLY PEOPLE)!!!!!!!!!!");
        }
    }

    /**
     * Runs all the systems
     *
     * @throws Exception
     */
    public void execute() throws Exception {
        if (cameraResponding) {
//            if(needImage){
                cameraLights.set(true);
                freeImage();
                getBestTarget();
                needImage = false;
//            }else{
//                cameraLights.set(false);
//            }
        }
    }

    /**
     * Sets weather or not to use CPU power to calculate images
     *
     * @param calculate. True if need calculations. False if not
     */
    public void setCameraMode(boolean calculate) {
        needImage = calculate;
    }

    public void liveWindow() {

    }

    public int sleepTime() {
        return 50;
    }

    /**
     * scoring criteria
     */
    private class Scores {

        double rectangularity;
        double aspectRatioVertical;
        double aspectRatioHorizontal;
    }

    /**
     * Gets and stores information from the target image
     */
    private class TargetReport {

        int verticalIndex;
        int horizontalIndex;
        boolean Hot;
        double totalScore;
        double leftScore;
        double rightScore;
        double tapeWidthScore;
        double verticalScore;
        double verticalPeremeter;
        double horizontalPeremeter;
        double location;
        double verticalWidth;
    };

    /**
     * Sees if a Vision has been created, if not it creates one
     *
     * @return the current class
     */
    public static Vision getInstance() {
        if (vision == null) {
            vision = new Vision();
        }
        return vision;
    }

    /**
     * Gets an image and uses color and small particle conversions to find
     * target.
     *
     * @throws NIVisionException
     */
    private void getImage() throws NIVisionException {
        startImageTime = Timer.getFPGATimestamp();
        image = null;
        try {
            image = camera.getImage();
        } catch (Exception e) {
            log.logError("Issue with getting image from the camera: " + e.getMessage());
        }
        
        thresholdImage = image.thresholdRGB(0, 255, 240, 255, 0, 255);   // keep only green objects
        filteredImage = thresholdImage.particleFilter(cc);           // filter out small particles 
    }

    /**
     * Finds the target and then calculates the center of the vertical tape.
     *
     * @throws NIVisionException
     */
    private void findCenterofTarget() throws NIVisionException {
        getImage();
        //iterate through each particle and score to see if it is a target
        scores = new Scores[filteredImage.getNumberParticles()];
        horizontalTargetCount = verticalTargetCount = 0;
        if (filteredImage.getNumberParticles() > 0) {
            for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) {
                ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                scores[i] = new Scores();
                //Score each particle on rectangularity and aspect ratio
                scores[i].rectangularity = scoreRectangularity(report);
                scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
                scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);
                imageLocation = report.center_mass_x;
                //Check if the particle is a horizontal target, if not, check if it's a vertical target
                if (scoreCompare(scores[i], false)) {
                    horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
                } else if (scoreCompare(scores[i], true)) {
                    verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
                }
            }
        }
    }

    /**
     * Finds the distance the camera is from the target. It then sees if the
     * aspect ratio and the distance match up to determine if the goal is a
     * normal goal or a hot goal.
     *
     * @throws NIVisionException
     */
    private void getBestTarget() throws NIVisionException {
        findCenterofTarget();
        target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
        target.verticalIndex = verticalTargets[0];
        for (int i = 0; i < verticalTargetCount; i++) {
            ParticleAnalysisReport verticalReport = filteredImage.getParticleAnalysisReport(verticalTargets[i]);
            for (int j = 0; j < horizontalTargetCount; j++) {
                ParticleAnalysisReport horizontalReport = filteredImage.getParticleAnalysisReport(horizontalTargets[j]);
                double horizWidth, horizHeight, vertWidth, vertPerimeter, horizPerimeter, leftScore, rightScore, tapeWidthScore, verticalScore, total, location;

                //Measure equivalent rectangle sides for use in score calculation
                horizWidth = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
                vertWidth = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                horizHeight = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                vertPerimeter = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, MeasurementType.IMAQ_MT_CONVEX_HULL_PERIMETER);
                horizPerimeter = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_CONVEX_HULL_PERIMETER);
                location = 180 - NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, MeasurementType.IMAQ_MT_FIRST_PIXEL_X);

                //Determine if the horizontal target is in the expected location to the left of the vertical target
                leftScore = ratioToScore(1.2 * (verticalReport.boundingRectLeft - horizontalReport.center_mass_x) / horizWidth);
                imageLocation = verticalReport.center_mass_x;
                cameraVerticalCount = verticalReport.boundingRectHeight;
                //Determine if the horizontal target is in the expected location to the right of the  vertical target
                rightScore = ratioToScore(1.2 * (horizontalReport.center_mass_x - verticalReport.boundingRectLeft - verticalReport.boundingRectWidth) / horizWidth);
                //Determine if the width of the tape on the two targets appears to be the same
                tapeWidthScore = ratioToScore(vertWidth / horizHeight);
                //Determine if the vertical location of the horizontal target appears to be correct
                verticalScore = ratioToScore(1 - (verticalReport.boundingRectTop - horizontalReport.center_mass_y) / (4 * horizHeight));
                total = leftScore > rightScore ? leftScore : rightScore;
                total += tapeWidthScore + verticalScore;

                //If the target is the best detected so far store the information about it
                if (total > target.totalScore) {
                    target.horizontalIndex = horizontalTargets[j];
                    target.verticalIndex = verticalTargets[i];
                    target.totalScore = total;
                    target.leftScore = leftScore;
                    target.rightScore = rightScore;
                    target.tapeWidthScore = tapeWidthScore;
                    target.verticalScore = verticalScore;
                    target.verticalPeremeter = vertPerimeter;
                    target.horizontalPeremeter = horizPerimeter;
                    target.verticalWidth = vertWidth;
                    target.location = location;
                }
            }
            //Determine if the best target is a Hot target
            target.Hot = hotOrNot(target);
        }

        if (verticalTargetCount > 0) {
            //Information about the target is contained in the "target" structure
            //To get measurement information such as sizes or locations use the
            //horizontal or vertical index to get the particle report as shown below
            ParticleAnalysisReport distanceReport = filteredImage.getParticleAnalysisReport(target.verticalIndex);
            double distance = computeDistance(filteredImage, distanceReport, target.horizontalIndex);
            imageDistance = distance * 12;
            if (target.Hot) {
                imageHotGoal = true;
            } else {
                imageHotGoal = false;
            }
        }
    }

    /**
     * frees all the images and saves space
     */
    private void freeImage() {
        try {
            filteredImage.free();
            thresholdImage.free();
            image.free();
        } catch (NIVisionException ex) {
            log.logError("Issue freeing the images from the camera: " + ex.getMessage());
        }
    }

    /**
     * Computes the estimated distance to a target using the height of the
     * particle in the image. For more information and graphics showing the math
     * behind this approach see the Vision Processing section of the
     * ScreenStepsLive documentation.
     *
     * @param image The image to use for measuring the particle estimated
     * rectangle
     * @param report The Particle Analysis Report for the particle
     * @param outer True if the particle should be treated as an outer target,
     * false to treat it as a center target
     * @return The estimated distance to the target in Inches. The equation
     * takes the total amount of pixels of the image (240) and then multiplies
     * if by the height of the projected image. This is then divided by the
     * actual height of the target. (multiplied by 12 for the feet to become
     * inches, then it is multiplied by the field of view of the camera). This
     * creates an imaginary triangle from which it is possible to determine
     * distance.
     */
    private double computeDistance(BinaryImage image, ParticleAnalysisReport report, int particleNumber) throws NIVisionException {
        double rectLong, height;
        double targetHeight;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        //using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
        //on skewed rectangles
        height = Math.min(report.boundingRectHeight, rectLong);
        boundingRectHeight = report.boundingRectHeight;
        targetHeight = 11.5;
        return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
    }

    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect
     * ratio for the target. This method uses the equivalent rectangle sides to
     * determine aspect ratio as it performs better as the target gets skewed by
     * moving to the left or right. The equivalent rectangle is the rectangle
     * with sides x and y where particle area= x*y and particle perimeter= 2x+2y
     *
     * @param image The image containing the particle to score, needed to
     * perform additional measurements
     * @param report The Particle Analysis Report for the particle, used for the
     * width, height, and particle number
     * @param outer	Indicates whether the particle aspect ratio should be
     * compared to the ratio for the inner target or the outer
     * @return The aspect ratio score (0-100)
     */
    private double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException {
        double rectLong, rectShort, aspectRatio, idealAspectRatio;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        idealAspectRatio = vertical ? (4.0 / 32) : (23.5 / 4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall

        //Divide width by height to measure aspect ratio
        if (report.boundingRectWidth > report.boundingRectHeight) {
            //particle is wider than it is tall, divide long by short
            aspectRatio = ratioToScore((rectLong / rectShort) / idealAspectRatio);
        } else {
            //particle is taller than it is wide, divide short by long
            aspectRatio = ratioToScore((rectShort / rectLong) / idealAspectRatio);
        }
        return aspectRatio;
    }

    /**
     * Compares scores to defined limits and returns true if the particle
     * appears to be a target
     *
     * @param scores The structure containing the scores to compare
     * @param outer True if the particle should be treated as an outer target,
     * false to treat it as a center target
     *
     * @return True if the particle meets all limits, false otherwise
     */
    boolean scoreCompare(Scores scores, boolean vertical) {
        boolean isTarget = true;

        isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
        if (vertical) {
            isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
        } else {
            isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
        }

        return isTarget;
    }

    /**
     * Computes a score (0-100) estimating how rectangular the particle is by
     * comparing the area of the particle to the area of the bounding box
     * surrounding it. A perfect rectangle would cover the entire bounding box.
     *
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
    double scoreRectangularity(ParticleAnalysisReport report) {
        if (report.boundingRectWidth * report.boundingRectHeight != 0) {
            return 100 * report.particleArea / (report.boundingRectWidth * report.boundingRectHeight);
        } else {
            return 0;
        }
    }

    /**
     * Converts a ratio with ideal value of 1 to a score. The resulting function
     * is piecewise linear going from (0,0) to (1,100) to (2,0) and is 0 for all
     * inputs outside the range 0-2
     */
    double ratioToScore(double ratio) {
        return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
    }

    /**
     * Takes in a report on a target and compares the scores to the defined
     * score limits to evaluate if the target is a hot target or not.
     *
     * Returns True if the target is hot. False if it is not.
     */
    boolean hotOrNot(TargetReport target) {
        boolean isHot = true;

        isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
        isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
        isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);

        return isHot;
    }

    /**
     * Gets an image and determines if there is a horizontal bar present.
     *
     * @return if the best goal is hot or not
     */
    public boolean isHotGoal() {
        return imageHotGoal;
    }

    /**
     * Gets an image and finds the reflective tape. After some fancy MATH it
     * returns the distance from the target in feet
     *
     * @return distance - how far away the target is from the camera in feet
     */
    private double getTargetDistance() {
        return imageDistance;
    }

    /**
     * Gets an image from the camera, then does a partical analysis and
     * determines where the center of the target is relative to the image
     *
     * @return location - the location of the best target. 180 is middle 0 is
     * right 360 is left
     */
    private int getLocation() {
        return imageLocation;
    }

    /**
     * The degrees are found by making an imaginary triangle. We already know
     * the distance and the location of the center of the target. First, the
     * inverse sin is
     *
     * @return the angle from camera to target in degrees
     */
    public double getDegrees() {
        pixelsToInches = cameraVerticalCount / TARGET_HEIGHT_INCHES;
        degrees = Math.toDegrees(MathUtils.asin(((getLocation() - CENTER_OF_CAMERA) / pixelsToInches) / (imageDistance)));
        return degrees;
    }

    public double getDistanceToGoal() {
        return Math.abs(Math.cos(getDegrees()) * getTargetDistance());
    }

    /**
     * Allows for the getDegrees and getDistance methods to return there updated
     * value
     */
    public double getLastImageTime() {
        return (Timer.getFPGATimestamp() - startImageTime);
    }

    /**
     * Sets weather to store an image or release it
     */
    public void saveImage() {
        if (ds.isEnabled()) {
            logWriter.writeImage(image);
            log.logMessage("Image has ben saved");
        }
    }
    
    public void logInfo() {
        log.logMessage("Dist to goal: " + getDistanceToGoal());
        log.logMessage("Dist to Target: " + getTargetDistance());
        log.logMessage("Hot Target: " + isHotGoal());
        log.logMessage("Average Runtime: " + getAverageRuntime() + "seconds");
    }
}
