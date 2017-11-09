package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia", group = "Iterative Opmode")
public class Vuforia extends LinearOpMode
{
	VuforiaLocalizer vuforiaLocalizer;
	VuforiaLocalizer.Parameters parameters;
	VuforiaTrackables visionTargets;
	VuforiaTrackable relicVuMark;
	VuforiaTrackableDefaultListener listener;

    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicVuMark);
	OpenGLMatrix lastKnownLocation;
	OpenGLMatrix phoneLocation;
	
	private static final String VUFORIA_KEY = "AahcXtr/////AAAAGeTM6MJozUDjmFmQyvzpw18JQGmMgCEJS4/mut4gWK23MK4IlXByqZJODNPcUsLluTIPxylZ00ZT+dnztgAgULPHoPca6zxDfRrdHxZaK0rRhdkAubtyi0J3if7ZFxYlC32J2wpWYb0N7QvMO1KfsG5s7fU24IaeXZhK8MeoD6CmnJfaVsa4brdMv2lqy1BUeGikI9FJphmw/JtS9r0FNM0Hk5ditj1qSkiFSYzpdS28Owzlwqudf5ZovyF8GtZ1xcfCpP4GWA1I0SOLxrFsXV74LkjoQi0AGVmnL3EXScKPeGmZPJtbd8oG2AzUUzYWnCwTi1X0+hEccMqG4WB8FsWMzYiYNrmS4+HfGJMExtno";
	
	private float robotX = 0;
	private float robotY = 0;
	private float robotAngle = 0;
	
	public void runOpMode() throws InterruptedException
	{
		setupVuforia();
		
		lastKnownLocation = createMatrix(0,0,0,0,0,0);
		
		waitForStart();
		
		visionTargets.activate();
		while(opModeIsActive())
		{
			OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
		
			if(latestLocation !=null)
			{lastKnownLocation = latestLocation;}
				
			float[] coordinates = lastKnownLocation.getTranslation().getData();
			
			robotX = coordinates[0];
			robotY = coordinates[1];
			robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("VuMark", "%s visible", vuMark);
            }else{
                telemetry.addData("VuMark", "not visible");
            }
			telemetry.addData("Last Known Loacation", formatMatrix(lastKnownLocation));
			
			telemetry.update();
			idle();
		}
	}
	
	public void setupVuforia()
	{

		parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); //remove parameters to hide phone tracking
		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		parameters.useExtendedTracking = false; //extended tracking is quite inaccurate
		vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);
		
		visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

		relicVuMark = visionTargets.get(0);
		relicVuMark.setName("RelicRecovery");
		relicVuMark.setLocation(createMatrix(0,0,0,0,0,0));
		
		phoneLocation = createMatrix(0,0,0,0,0,0);
		
		listener = (VuforiaTrackableDefaultListener) relicVuMark.getListener();
		listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
		
	}
	
	public  OpenGLMatrix createMatrix(float x,float y,float z,float u,float v,float w)
	{
		return OpenGLMatrix.translation(x,y,z).
			multiplied(Orientation.getRotationMatrix(
				AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u,v,w));
	}
	
	public String formatMatrix(OpenGLMatrix matrix)
	{
		return matrix.formatAsTransform();
	}
}
