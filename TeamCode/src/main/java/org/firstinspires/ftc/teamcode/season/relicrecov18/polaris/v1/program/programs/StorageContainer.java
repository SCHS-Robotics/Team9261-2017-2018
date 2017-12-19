package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sage Creek Level Up on 12/13/2017.
 */

public class StorageContainer {
    //A simple box class
    private static ArrayList<Object> storage1 = new ArrayList<>();
    private static ArrayList<Object> storage2 = new ArrayList<>();
    private static ArrayList<Object> storage3 = new ArrayList<>();
    private static ArrayList<Object> storage4 = new ArrayList<>();
    private static ArrayList<Object> storage5 = new ArrayList<>();

    public StorageContainer() {

    }

    public void add(Object d,int type) {
        if(type == 1) {
            storage1.add(d);
        }
        else if(type == 2) {
            storage2.add(d);
        }
        else if(type == 3) {
            storage3.add(d);
        }
        else if(type == 4) {
            storage4.add(d);
        }
        else if(type == 5) {
            storage5.add(d);
        }
    }

    public Object get(int d,int type) {
        if(type == 1) {
            return storage1.get(d);
        }
        else if(type == 2) {
            return storage2.get(d);
        }
        else if(type == 3) {
            return storage3.get(d);
        }
        else if(type == 4) {
            return storage4.get(d);
        }
        else if(type == 5) {
            return storage5.get(d);
        }
        else {
            return 0;
        }
    }
    public void clear(int type) {
        if (type == 1) {
            storage1 = new ArrayList<>();
        }
        else if (type == 2) {
            storage2 = new ArrayList<>();
        }
        else if (type == 3) {
            storage3 = new ArrayList<>();
        }
        else if (type == 4) {
            storage4 = new ArrayList<>();
        }
        else if (type == 5) {
            storage5 = new ArrayList<>();
        }
    }

    public double getMean(int type) {
        if(type == 1) {
            return mean(storage1);
        }
        else if(type == 2) {
            return mean(storage2);
        }
        else if(type == 3) {
            return mean(storage3);
        }
        else if(type == 4) {
            return mean(storage4);
        }
        else if(type == 5) {
            return mean(storage5);
        }
        else {
            return 0;
        }
    }

    private double mean(List<Object> vals) {
        double sum = 0;
        for (Object val : vals) {
            sum += (double) val;
        }
        return sum / vals.size();
    }
}