package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.drive.MecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.VisionEasyOpenCV;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * RoboRaisers # 21386 Autonomous using EasyOpenCV & for dropping preloaded cone, picking and dropping 2 cones and park
 */
@Autonomous(name = "RoboRaiders Autonomous Full")
public class AutoOpMode extends LinearOpMode{
    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public enum ALLIANCE_POSITION{
        ALLIANCE_LEFT,
        ALLIANCE_RIGHT
    }

    public static START_POSITION startPosition;
    public static ALLIANCE_POSITION startAlliancePosition;


    public VisionEasyOpenCV visionEasyOpenCV;
    private OpenCvCamera camera;
    public DriveTrain driveTrain;
    private String webcamName = "Webcam 1";
    private DcMotor Arm;
    private Servo Claw;

    private  double STRAFE_DISTANCE = 24;
    private double CAMERA_OFFSET = 4 ;
    @Override
    public void runOpMode() throws InterruptedException {
        /*Create your Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        visionEasyOpenCV = new VisionEasyOpenCV();

        //Initialize claw and slide motor
        Arm = hardwareMap.get(DcMotor.class, "motor");
        Claw = hardwareMap.get(Servo.class, "Claw");
        double ServoPosition;
        double ServoSpeed;
        ServoPosition = 1;
        ServoSpeed = 1;
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);


        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        //selectAlliancePosition();

        // Initiate Camera on INIT.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        camera.setPipeline(visionEasyOpenCV);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //Build Autonomous trajectory to be used based on starting position selected
        buildAllianceAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run visionEasyOpenCV.getPosition() and keep watching for the identifier in the Signal Cone.
            telemetry.clearAll();
            telemetry.addData("Start RoboRaiders Autonomous Mode adopted for Team","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Alliance Position", startAlliancePosition);
            telemetry.addData("Camera Detected: ", visionEasyOpenCV.getPosition());
            telemetry.update();
        }
        //Set DEFAULT as LEFT
        VisionEasyOpenCV.ParkingPosition position = VisionEasyOpenCV.ParkingPosition.LEFT;

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Start is pressed

            //Set the arm to low junction height
            Arm.setTargetPosition(1000);
            Arm.setPower(0.3);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Arm.isBusy()) {
                telemetry.addData("Current Position", Arm.getCurrentPosition());
                telemetry.addData("Target Position", Arm.getTargetPosition());
                telemetry.update();
            }


            //get the camera position
            position = visionEasyOpenCV.getPosition();
            telemetry.addData("Camera Parking Position: ", visionEasyOpenCV.getPosition());


            //Build parking trajectory based on last detected target by visionEasyOpenCV
            buildAllianceParking(position);
            //stop camera streaming
            //camera.stopStreaming();

            //run Autonomous trajectory
            //runAllianceAutoAndParking(position);
            runAutoAndParking(position);
        }

        //Trajectory is completed, display Parking complete
        parkingComplete(position);
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto, trajectoryParking ;
    TrajectorySequence trajectoryParkingLEFT ;
    TrajectorySequence trajectoryParkingRIGHT ;
    TrajectorySequence trajectoryParkingCENTER ;

    TrajectorySequence trajectoryAutoNew ;

    //Initialize any other Pose2d's as desired
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d pickConePose;
    Pose2d dropConePose0, dropConePose1, dropConePose2;
    Pose2d parkPose;
    Pose2d dropConePose;
    Pose2d firstPose;


    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(-12, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(-11, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(-10, 12, Math.toRadians(225)); //Choose the pose to move to the stack of cones
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-70, -40, Math.toRadians(0));//Starting pose
                firstPose = new Pose2d(-30, -36, Math.toRadians(0));
                dropConePose = new Pose2d(-24, -35, Math.toRadians(90));
                midWayPose = new Pose2d(-41, -35, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(-12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(-12, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(-11, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(-10, -12, Math.toRadians(135)); //Choose the pose to move to the stack of cones
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(12, -12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(11, -12, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(10, -15, Math.toRadians(45)); //Choose the pose to move to the stack of cones
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickConePose = new Pose2d(12, 55, Math.toRadians(90)); //Choose the pose to move to the stack of cones
                dropConePose0 = new Pose2d(12, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                dropConePose1 = new Pose2d(11, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                dropConePose2 = new Pose2d(10, 12, Math.toRadians(315)); //Choose the pose to move to the stack of cones
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(firstPose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone(0); //Drop preloaded Cone
                })
                //Uncomment following line to stop reduction in speed. And move to the position after which you want to stop reducing speed.
                //.resetVelConstraint()

                .lineToLinearHeading(midWayPose)
                /*
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone(1); //Pick top cone from stack
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose1)
                .addDisplacementMarker(() -> {
                    dropCone(1); //Drop cone on junction
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone(2); //Pick second cone from stack
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose2)
                .addDisplacementMarker(() -> {
                    dropCone(2); //Drop cone on junction
                })
                .lineToLinearHeading(midWayPose)
                */
                .build();
    }

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAllianceAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-70, -36 - CAMERA_OFFSET, Math.toRadians(0));//Starting pose
                dropConePose = new Pose2d(-24, -41, Math.toRadians(90));
                midWayPose = new Pose2d(-40, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                break;
            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                break;
            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                dropConePose = new Pose2d(12, -55, Math.toRadians(270)); //Choose the pose to move to the stack of cones
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park

        trajectoryAutoNew = driveTrain.trajectorySequenceBuilder(initPose)
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .forward(46)
                .back(12)
                .strafeLeft(8)
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    //moveSlide(2700); //Drop preloaded Cone
                    Arm.setTargetPosition(2800);
                    Arm.setPower(0.3);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (Arm.isBusy()) {
                        telemetry.addData("Current Position", Arm.getCurrentPosition());
                        telemetry.addData("Target Position", Arm.getTargetPosition());
                    }
                })
                .forward(12)
                .addDisplacementMarker(() -> {
                    dropCone(0); //Drop preloaded Cone
                })
                .lineToLinearHeading(midWayPose)
                .build();
        telemetry.update();


        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                //.lineToLinearHeading(firstPose)
                //Uncomment following line to slow down turn if needed.
                .setVelConstraint(getVelocityConstraint(30 /* Slower Velocity*/, 15 /*Slower Angular Velocity*/, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(dropConePose)
                .addDisplacementMarker(() -> {
                    dropCone(0); //Drop preloaded Cone
                })
                //Uncomment following line to stop reduction in speed. And move to the position after which you want to stop reducing speed.
                //.resetVelConstraint()

                .lineToLinearHeading(midWayPose)
                /*
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone(1); //Pick top cone from stack
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose1)
                .addDisplacementMarker(() -> {
                    dropCone(1); //Drop cone on junction
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickConePose)
                .addDisplacementMarker(() -> {
                    pickCone(2); //Pick second cone from stack
                })
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(dropConePose2)
                .addDisplacementMarker(() -> {
                    dropCone(2); //Drop cone on junction
                })
                .lineToLinearHeading(midWayPose)
                */
                .build();
    }

    //Build parking trajectory based on target detected by visionEasyOpenCV
    public void buildParking(VisionEasyOpenCV.ParkingPosition position){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(position){
                    case LEFT: parkPose = new Pose2d(-40, 60, Math.toRadians(0)); break; // Location 1
                    case CENTER: parkPose = new Pose2d(-38, 35, Math.toRadians(0)); break; // Location 2
                    case RIGHT: parkPose = new Pose2d(-40, 11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch(position){
                    case LEFT: parkPose = new Pose2d(-40, -11, Math.toRadians(0)); break; // Location 1
                    case CENTER: parkPose = new Pose2d(-38, -35, Math.toRadians(0)); break; // Location 2
                    case RIGHT: parkPose = new Pose2d(-40, -60, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_LEFT:
                switch(position){
                    case LEFT: parkPose = new Pose2d(40, -60, Math.toRadians(180)); break; // Location 1
                    case CENTER: parkPose = new Pose2d(38, -35, Math.toRadians(180)); break; // Location 2
                    case RIGHT: parkPose = new Pose2d(40, -11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(position){
                    case LEFT: parkPose = new Pose2d(40, 11, Math.toRadians(180)); break; // Location 1
                    case CENTER: parkPose = new Pose2d(38, 35, Math.toRadians(180)); break; // Location 2
                    case RIGHT: parkPose = new Pose2d(40, 60, Math.toRadians(180)); break; // Location 3
                }
                break;
        }
        
        trajectoryParking = driveTrain.trajectorySequenceBuilder(midWayPose)
                .lineToLinearHeading(parkPose)
                .build();
    }

    public void buildAllianceParking(VisionEasyOpenCV.ParkingPosition position){
        //Build trajectory for parking based on the Camera color detected and position identified as LEFT, CENTER, RIGHT
        trajectoryParkingLEFT = driveTrain.trajectorySequenceBuilder(midWayPose)
                .strafeLeft(STRAFE_DISTANCE + CAMERA_OFFSET)
                .build();

        trajectoryParkingRIGHT = driveTrain.trajectorySequenceBuilder(midWayPose)
                .strafeRight(STRAFE_DISTANCE)
                .build();

        trajectoryParkingCENTER = driveTrain.trajectorySequenceBuilder(midWayPose)
                .forward(1)
                .build();

     }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(VisionEasyOpenCV.ParkingPosition position){
        telemetry.setAutoClear(false);
        telemetry.addData("Running RoboRaiders Autonomous Mode adopted for Team:","TEAM NUMBER");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        //driveTrain.followTrajectorySequence(trajectoryAuto);
        driveTrain.followTrajectorySequence(trajectoryAutoNew);
        //driveTrain.followTrajectorySequence(trajectoryParking);

        //Run trajectory based on camera direction detected
        switch (position) {
            case LEFT: driveTrain.followTrajectorySequence(trajectoryParkingLEFT); break;
            case CENTER: driveTrain.followTrajectorySequence(trajectoryParkingCENTER); break;
            case RIGHT: driveTrain.followTrajectorySequence(trajectoryParkingRIGHT);break;
        }
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        /*TODO: Add code to pick Cone 1 from stack*/
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(int coneCount){
        /*TODO: Add code to drop cone on junction*/
        Claw.setPosition(.35);
        //ServoPosition += ServoSpeed;
        telemetry.addData("Opening","Claw is opening");

        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    public void moveSlide(int slide_pos){
        /*Move slide to required position*/
        Arm.setTargetPosition(slide_pos);
        Arm.setPower(0.3);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            telemetry.addData("Current Position", Arm.getCurrentPosition());
            //telemetry.update();
        }
        telemetry.update();
    }

    public void parkingComplete(VisionEasyOpenCV.ParkingPosition position){
        Claw.setPosition(0.65);
        Arm.setTargetPosition(0);
        Arm.setPower(0.3);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm.isBusy()) {
            telemetry.addData("Current Position", Arm.getCurrentPosition());
        }
        telemetry.addData("Parked in Location", position);
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing RoboRaiders Autonomous Mode adopted for Team:","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //Method to select starting position using X, Y buttons on gamepad
    public void selectAlliancePosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing RoboRaiders Autonomous Mode adopted for Team:","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Alliance Position using XY Keys on gamepad 1:","");
            telemetry.addData("    Left Side   ", "(X)");
            telemetry.addData("    Right Side ", "(Y)");
            if(gamepad1.x){
                startAlliancePosition = ALLIANCE_POSITION.ALLIANCE_LEFT;
                break;
            }
            if(gamepad1.y){
                startAlliancePosition = ALLIANCE_POSITION.ALLIANCE_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

}

