package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

/**
 * Created by Sage Creek Level Up on 12/10/2017.
 */

public class DefaultExceptionHandler implements Thread.UncaughtExceptionHandler {
    private Activity activity;
    private Context context;

    public DefaultExceptionHandler(Activity activity, Context context) {
        this.activity = activity;
        this.context = context;
    }

    @Override
    public void uncaughtException(Thread thread, Throwable ex) {
        //Log the exception so we know what happened
        Log.e("FtcRobotController","Unhandled fatal exception - restarting app", ex);
        try {
            //set up the Intent to restart the app
            Intent intent = new Intent(activity, FtcRobotControllerActivity.class);
            intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_CLEAR_TASK | Intent.FLAG_ACTIVITY_NEW_TASK);
            PendingIntent pendingIntent = PendingIntent.getActivity(context,0,intent,PendingIntent.FLAG_ONE_SHOT);

            //schedule the restart in two seconds
            AlarmManager mgr = (AlarmManager) context.getSystemService(Context.ALARM_SERVICE);
            mgr.set(AlarmManager.RTC,System.currentTimeMillis() + 2000, pendingIntent);
            //finish the activity and close the dead app
            activity.finish();
            System.exit(2);
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }
}
