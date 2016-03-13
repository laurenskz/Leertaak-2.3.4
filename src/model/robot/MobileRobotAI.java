package model.robot;

import model.virtualmap.OccupancyMap;

import java.io.PipedInputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.PipedOutputStream;
import java.io.IOException;

import java.util.Arrays;
import java.util.Random;
import java.util.StringTokenizer;

/**
 * Title    :   The Mobile Robot Explorer Simulation Environment v2.0
 * Copyright:   GNU General Public License as published by the Free Software Foundation
 * Company  :   Hanze University of Applied Sciences
 *
 * @author Dustin Meijer        (2012)
 * @author Alexander Jeurissen  (2012)
 * @author Davide Brugali       (2002)
 * @version 2.0
 */

public class MobileRobotAI implements Runnable {

    private final OccupancyMap map;
    private final MobileRobot robot;
    private double robotX,robotY,robotWidth,robotHeight;

    private boolean running;

    public MobileRobotAI(MobileRobot robot, OccupancyMap map) {
        this.map = map;
        this.robot = robot;
    }

    /**
     * In this method the gui.controller sends commands to the robot and its devices.
     * At the moment all the commands are hardcoded.
     * The exercise is to let the gui.controller make intelligent decisions based on
     * what has been discovered so far. This information is contained in the OccupancyMap.
     */
    public void run() {
        String result;
        this.running = true;
        double position[] = new double[3];
        double measures[] = new double[360];
        while (running) {
            try {

                PipedInputStream pipeIn = new PipedInputStream();
                BufferedReader input = new BufferedReader(new InputStreamReader(pipeIn));
                PrintWriter output = new PrintWriter(new PipedOutputStream(pipeIn), true);

                robot.setOutput(output);

//      ases where a variable value is never used after its assignment, i.e.:
                System.out.println("intelligence running");
                robotX = robot.getPlatform().getShape().getBounds().getX();
                robotY = robot.getPlatform().getShape().getBounds().getY();
                robotHeight = robot.getPlatform().getShape().getBounds().getHeight();
                robotWidth = robot.getPlatform().getShape().getBounds().getWidth();
                

                this.running = false;
            } catch (IOException ioe) {
                System.err.println("execution stopped");
                running = false;
            }
        }

    }

    private void rotateTo(double degrees,BufferedReader input,double[] position){
        try{
            double toMove = degrees - position[2];
            int degreesAsInt = (int) Math.round(toMove);
            getPosition(position,input);
            if(degreesAsInt<0){
                rotateLeft(input,-degreesAsInt);
            }else if(degreesAsInt>0){
                rotateRight(input,degreesAsInt);
            }
        }catch(IOException e){
            e.printStackTrace();
        }
    }

    private  void moveTo(double[] position, double[] currentPosition, BufferedReader input){
        try {
            getPosition(currentPosition,input);
        } catch (IOException e) {
            e.printStackTrace();
        }
        double dX = position[0] - currentPosition[0];//Aanliggende zijde
        double dY = position[1] - currentPosition[1];//Overstaande zijde
        double degrees = getDegrees(dX, dY);
        moveRobot(currentPosition, input, dX, dY, degrees);
    }

    private void moveRobot(double[] currentPosition, BufferedReader input, double dX, double dY, double degrees) {
        rotateTo(degrees,input,currentPosition);
        int distance = (int)Math.sqrt(dX*dX+dY*dY);
        move(input,distance);
    }

    private static double getDegrees(double dX, double dY) {
        double tan = dY /dX;
        double radians = Math.atan(tan);
        double degrees = Math.toDegrees(radians);
        if(dX<0)
            degrees+=180d;
        else if(dY<0)
            degrees+=360d;
        return degrees;
    }

    private int[] getWallBlockToFollow(){
        for (int i = 0; i < map.getGrid().length; i++) {
            for (int j = 0; j < map.getGrid()[i].length; j++) {
                if(map.getGrid()[i][j]==map.getObstacle()){
                    if(!map.isFollowed(i,j)){

                        return new int[]{i,j};
                    }
                }
            }
        }
        return null;
    }

    private int[][] squaresOnRoute(int[] position, double[] currentPosition, BufferedReader input) {
        return null;
    }



    private boolean get(){
        return true;
    }

    private void rotateLeft(BufferedReader input, int degrees) throws IOException {
        robot.sendCommand("P1.ROTATELEFT "+degrees);
        input.readLine();
    }

    private void rotateRight(BufferedReader input, int degrees) throws IOException {
        robot.sendCommand("P1.ROTATERIGHT "+degrees);
        input.readLine();
    }

    private void move(BufferedReader input, int distance) {
        try {
            if (distance < 0) {
                moveBackwards(input, -distance);
            } else {
                moveForward(input, distance);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void moveBackwards(BufferedReader input, int distance) throws IOException {
        robot.sendCommand("P1.MOVEBW " + distance);
        input.readLine();
    }

    private void moveForward(BufferedReader input, int distance) throws IOException {
        robot.sendCommand("P1.MOVEFW " + distance);
        input.readLine();
    }

    private void scan(double[] position, double[] measures, BufferedReader input) throws IOException {
        String result;
        robot.sendCommand("L1.SCAN");
        result = input.readLine();
        parseMeasures(result, measures);
        map.drawLaserScan(position, measures);
    }

    private void getPosition(double[] position, BufferedReader input) throws IOException {
        String result;
        robot.sendCommand("R1.GETPOS");
        result = input.readLine();
        parsePosition(result, position);
    }

    private void parsePosition(String value, double position[]) {
        int indexInit;
        int indexEnd;
        String parameter;

        indexInit = value.indexOf("X=");
        parameter = value.substring(indexInit + 2);
        indexEnd = parameter.indexOf(' ');
        position[0] = Double.parseDouble(parameter.substring(0, indexEnd));

        indexInit = value.indexOf("Y=");
        parameter = value.substring(indexInit + 2);
        indexEnd = parameter.indexOf(' ');
        position[1] = Double.parseDouble(parameter.substring(0, indexEnd));

        indexInit = value.indexOf("DIR=");
        parameter = value.substring(indexInit + 4);
        position[2] = Double.parseDouble(parameter);
    }

    private void parseMeasures(String value, double measures[]) {
        for (int i = 0; i < 360; i++) {
            measures[i] = 100.0;
        }

        if (value.length() >= 5) {
            value = value.substring(5);  // removes the "SCAN " keyword

            StringTokenizer tokenizer = new StringTokenizer(value, " ");

            double distance;
            int direction;
            while (tokenizer.hasMoreTokens()) {
                distance = Double.parseDouble(tokenizer.nextToken().substring(2));
                direction = (int) Math.round(Math.toDegrees(Double.parseDouble(tokenizer.nextToken().substring(2))));
                if (direction == 360) {
                    direction = 0;
                }
                measures[direction] = distance;
                // Printing out all the degrees and what it encountered.
//                System.out.println("direction = " + direction + " distance = " + distance);
            }
        }
    }

    public static void main(String[] args) {
        int[][] deltas = new int[][]{
                new int[]{-1,-1},
                new int[]{-2,-1},
                new int[]{-1,0},
                new int[]{-1,1},
                new int[]{-1,2},
                new int[]{0,1},
                new int[]{1,1},
                new int[]{2,1},
                new int[]{1,0},
                new int[]{2,-1},
                new int[]{1,-1},
                new int[]{0,-1},
        };
        for (int i = 0; i < deltas.length; i++) {
            System.out.printf("dX = %d dY = %d\n",deltas[i][0],deltas[i][1]);
            System.out.printf("degrees = %f\n",getDegrees(deltas[i][0],deltas[i][1]));
        }
    }
}
