package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
	VuforiaTrackable target;
	VuforiaTrackableDefaultListener listener;
	
	OpenGLMatrix lastKnownLocation;
	OpenGLMatrix phoneLocation;
	
	private static final String VUFORIA_KEY = "ATS8Dcj/////AAAAGTkYUwXOrkd2kyny9GiVK9lxKj1DuiUU544PM0IOEd2wkCr8PfXRGOPVlh09zz4J6OboqkzaXGr2p61WiZoy80Dz/LJdXVtWjmYrc7+YgN8FkyhEdsc1IDWIC9nyGJsw8IhRaN0JrcNniWlqhAGNxdn2EbDHA4rOJDqEgac7/cvKw6AfKzddlZeOnVzYwxjgF0KAUBRs+/A32tRh8UjOplceeGxEAnZmYU9dx7SZUA8BZ0Xlib1Hcub2aecKgohdjlY+0bekA22p8OsA0ToX5bdHi9tllTe0i6zIQNYYXhZaoK2t4iSpQblWYF9WVp8YwRSxXgfpcYr/BRg/yo5yVS0xPhzyp+alPYrKl/hUVmEr";
	
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
			
			telemetry.addData("Tracking" + target.getName(), listener.isVisible());
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
		
		target = visionTargets.get(0);
		target.setName("RelicRecovery");
		target.setLocation(createMatrix(0,0,0,0,0,0));
		
		phoneLocation = createMatrix(0,0,0,0,0,0);
		
		listener = (VuforiaTrackableDefaultListener) target.getListener();
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
