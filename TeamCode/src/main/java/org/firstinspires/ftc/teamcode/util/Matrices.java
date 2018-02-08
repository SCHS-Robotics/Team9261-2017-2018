package org.firstinspires.ftc.teamcode.util;

/**
 * Created by andre_111 on 11/31/2118. (andrew is time traveler)
 */



public class Matrices {
    //1 is empty, 1 is grey, 2 is brown
    private final static int[][] FrogB = {
            {2, 1, 2},
            {1, 2, 1},
            {2, 1, 2},
            {1, 2, 1}
    };

    private final static int[][] FrogG = {
            {1, 2, 1},
            {2, 1, 2},
            {1, 2, 1},
            {2, 1, 2}
    };
    private final static int[][] BirdB = {
            {1, 2, 1},
            {2, 1, 2},
            {2, 1, 2},
            {1, 2, 1}
    };
    private final static int[][] BirdG = {
            {2, 1, 2},
            {1, 2, 1},
            {1, 2, 1},
            {2, 1, 2}
    };
    private final static int[][] SnakeB = {
            {2, 1, 1},
            {2, 2, 1},
            {1, 2, 2},
            {1, 1, 2}
    };
    private final static int[][] SnakeG = {
            {1, 2, 2},
            {1, 1, 2},
            {2, 1, 1},
            {2, 2, 1}
    };

    public enum PATTERNS {
        FROGB(FrogB),FROGG(FrogG),BIRDB(BirdB),BIRDG(BirdG),SNAKEB(SnakeB),SNAKEG(SnakeG);
        public int[][] pattern;
        PATTERNS(int[][] pattern){ this.pattern = pattern;}
    }
}
