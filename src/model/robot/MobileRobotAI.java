package model.robot;

import model.virtualmap.OccupancyMap;

import java.awt.event.ActionEvent;
import java.io.PipedInputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.PipedOutputStream;
import java.io.IOException;

import java.util.*;

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

    public static final int MOVE_DISTANCE = 1;
    public static final int MIN_STEPS_SINCE_RIGHT = 27;
    private final OccupancyMap map;
    private final MobileRobot robot;
    private double robotX, robotY, robotWidth, robotHeight;

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
                robotHeight = robot.getPlatform().getShape().getBounds().height;
                robotWidth = robot.getPlatform().getShape().getBounds().width;
//                double xStart = 200,yStart = 200, xDest = 400,yDest = 400;
//                Set<int[]>visited = pointsBetween(xStart,yStart,xDest,yDest,xStart+90,yStart+30,xDest+90,yDest+30);
                mazeWallFollower(position, measures, input);
                this.running = false;
            } catch (IOException ioe) {
                System.err.println("execution stopped");
                running = false;
            }
        }

    }

    private double distanceBetween(double[] a, double[] b){
        double dX = Math.abs(b[0]-a[0]),dY=Math.abs(a[1]-b[1]);
        return Math.sqrt(dX*dX+dY*dY);
    }


    private void mazeWallFollower(double[] position, double[] measures, BufferedReader input) throws IOException{
        getPosition(position,input);

        double[] startPosition = clone(position);
        int degrees = (int)Math.round(position[2]);
        scan(position,measures,input);
        int stepsSinceRight = Integer.MIN_VALUE;
        boolean running = true;
        boolean leftStartArea = false;
        while(running){

            while(safeMove(position, MOVE_DISTANCE)){
                getPosition(position,input);
                if(!leftStartArea)
                    if(distanceBetween(position,startPosition)>80)leftStartArea=true;
                if(leftStartArea&&distanceBetween(position,startPosition)<30)running=false;
                move(input,MOVE_DISTANCE);
                stepsSinceRight++;
                getPosition(position,input);
                // If we can go right, we do that
                if(stepsSinceRight> MIN_STEPS_SINCE_RIGHT &&canGoRight(position,input)){
                    // Turn 90 degrees to the right
                    scan(position,measures,input);
                    if(canGoRight(position,input)){
                        degrees = (degrees+90)%360;
                        rotateTo(degrees,input,position);
                        getPosition(position, input);
                        stepsSinceRight = 0;
                    }
                }
            }
            getPosition(position,input);
            scan(position,measures,input);
            if(safeMove(position,MOVE_DISTANCE))continue;
            if(stepsSinceRight>=0&&canGoRight(position,input)){
                degrees = (degrees+90)%360;
                stepsSinceRight = 0;
            } else{
                if(stepsSinceRight<0)stepsSinceRight=0;
                degrees = (degrees-90)%360;
            }
            rotateTo(degrees,input,position);
        }
    }

    private boolean canGoRight(double[] position,BufferedReader input){
        double[] clonedPosition = clone(position);
        clonedPosition[2] = position[2]+90;
        return safeMove(clonedPosition,MOVE_DISTANCE);
    }

    private double[] clone(double[] ar) {
        return Arrays.copyOf(ar,ar==null?0:ar.length);
    }


    private boolean safeMove(double[] position, int distance) {
        double[] positionClone = clone(position);
        positionClone[2] = Math.round(position[2]);
        double [] destination = translateWithAngleAndDistance(position,distance);
        Set<int[]> pointsOnRoute = pointsBetween(positionClone, distance);
        for (int[] ints : pointsOnRoute) {
            if (map.getGridPoint(ints[0], ints[1], map.getObstacle()) != map.getEmpty() && map.getGridPoint(ints[0], ints[1], map.getObstacle()) != map.getRobot()) {
                return false;
            }
        }
        return true;
    }


    private Set<int[]> pointsBetween(double[] position, int distance) {
        double[][] startBounds = getRobotBounds(position);
        double[] endPosition = translateWithAngleAndDistance(position, distance);
        double[][] endBounds = getRobotBounds(endPosition);
        return pointsBetween(startBounds[2][0], startBounds[2][1], endBounds[0][0], endBounds[0][1], startBounds[3][0], startBounds[3][1], endBounds[1][0], endBounds[1][1]);
    }

    private double[] translateWithAngleAndDistance(double[] position, int distance) {
        double[] endPosition = new double[3];
        double degrees = Math.toRadians(position[2]);
        double dx = Math.cos(degrees) * distance;
        double dy = Math.sin(degrees) * distance;
        endPosition[0] = position[0] + dx;
        endPosition[1] = position[1] + dy;
        endPosition[2] = position[2];
        return endPosition;
    }

    private Set<int[]> pointsBetween(double x00, double y00, double x01, double y01, double x10, double y10, double x11, double y11) {
        Set<int[]> visitedPoints = raytrace(x00, y00, x01, y01, map);
        visitedPoints.addAll(raytrace(x00, y00, x10, y10, map));
        visitedPoints.addAll(raytrace(x01, y01, x11, y11, map));
        visitedPoints.addAll(raytrace(x10, y10, x11, y11, map));
        addPointsBetweenBounds(visitedPoints);
        return visitedPoints;
    }

    private void addPointsBetweenBounds(Set<int[]> visitedPoints) {
        int ySize = map.getMapHeight() / map.getCellDimension();//Make an array for all y's
        int[][] xValues = new int[ySize][2];//Every y should have two x's between which all blocks should be added
        initializeArray(xValues);
        setXminAndMax(visitedPoints, xValues);
        addVisitedPoints(visitedPoints, xValues);
    }

    private void addVisitedPoints(Set<int[]> visitedPoints, int[][] xValues) {
        for (int i = 0; i < xValues.length; i++) {
            int[] xMinAndMax = xValues[i];
            if (xMinAndMax[0] < 0 || xMinAndMax[1] < 0) continue;
            for (int j = xMinAndMax[0]; j < xMinAndMax[1]; j++) {
                visitedPoints.add(new int[]{j, i});
            }
        }
    }

    private void setXminAndMax(Set<int[]> visitedPoints, int[][] xValues) {
        for (int[] point : visitedPoints) {
            if (point[1] < 0 || point[1] >= xValues.length) continue;
            if (point[0] == -1) continue;
            if (xValues[point[1]][0] == -1) xValues[point[1]][0] = point[0];
            if (xValues[point[1]][0] > point[0]) xValues[point[1]][0] = point[0];
            if (xValues[point[1]][1] < point[0]) xValues[point[1]][1] = point[0];
        }
    }

    private void initializeArray(int[][] xValues) {
        for (int[] xValue : xValues) {//In
            for (int i = 0; i < xValue.length; i++) {
                xValue[i] = -1;
            }
        }
    }

    private double[][] getRobotBounds(double[] position) {
        double vx = robotWidth + robotY;
        double vy = robotHeight / 2;

        double degrees = Math.toRadians(360d - position[2]);

        double[] leftFront = getLeftFront(position, vx, vy, degrees);
        double[] rightFront = getRightFront(vy, degrees, leftFront);
        double[] leftBack = getLeftBack(degrees, leftFront);
        double[] rightBack = getRightBack(vy, degrees, leftBack);

        return new double[][]{leftFront, rightFront, leftBack, rightBack};
    }

    private double[] getRightBack(double vy, double degrees, double[] leftBack) {
        double[] rightBack = new double[2];
        rightBack[0] = leftBack[0] + (2 * vy) * Math.sin(degrees);
        rightBack[1] = leftBack[1] + (2 * vy) * Math.cos(degrees);
        return rightBack;
    }

    private double[] getLeftBack(double degrees, double[] leftFront) {
        double[] leftBack = new double[2];
        leftBack[0] = leftFront[0] - robotWidth * Math.cos(degrees);
        leftBack[1] = leftFront[1] + robotWidth * Math.sin(degrees);
        return leftBack;
    }

    private double[] getRightFront(double vy, double degrees, double[] leftFront) {
        double[] rightFront = new double[2];
        rightFront[0] = leftFront[0] + (2 * vy) * Math.sin(degrees);
        rightFront[1] = leftFront[1] + (2 * vy) * Math.cos(degrees);
        return rightFront;
    }

    private double[] getLeftFront(double[] position, double vx, double vy, double degrees) {
        double[] leftFront = new double[2];
        leftFront[0] = position[0] + vx * Math.cos(degrees) - vy * Math.sin(degrees);
        leftFront[1] = position[1] - vx * Math.sin(degrees) - vy * Math.cos(degrees);
        return leftFront;
    }

    private void rotateTo(double degrees, BufferedReader input, double[] position) {
        try {
            getPosition(position, input);
            double toMove = degrees - position[2];
            int degreesAsInt = getDegreesAsInt(toMove);
            rotate(input, degreesAsInt);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void rotate(BufferedReader input, int degreesAsInt) throws IOException {
        if (degreesAsInt < 0) {
            rotateLeft(input, -degreesAsInt);
        } else if (degreesAsInt > 0) {
            rotateRight(input, degreesAsInt);
        }
    }

    private int getDegreesAsInt(double toMove) {
        int degreesAsInt = (int) Math.round(toMove);
        if(degreesAsInt>180){
            degreesAsInt-=360;
        }
        if(degreesAsInt<-180){
            degreesAsInt+=360;
        }
        return degreesAsInt;
    }


    private void rotateLeft(BufferedReader input, int degrees) throws IOException {
        robot.sendCommand("P1.ROTATELEFT " + degrees);
        input.readLine();
    }

    private void rotateRight(BufferedReader input, int degrees) throws IOException {
        robot.sendCommand("P1.ROTATERIGHT " + degrees);
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
        robot.sendCommand("S1.SCAN");
        result = input.readLine();
        parseMeasures(result, measures);
        robot.sendCommand("L1.SCAN");
        result = input.readLine();
        double[] secondMeasures = new double[measures.length];
        parseMeasures(result, secondMeasures);
        merge(measures,secondMeasures);
        map.drawLaserScan(position, measures);
    }

    private void merge(double[] a , double[] b){
        for (int i = 0; i < a.length; i++) {
            if(b[i]<a[i])a[i] = b[i];
        }
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

    private static Set<int[]> raytrace(double x0, double y0, double x1, double y1, OccupancyMap map) {
        x0 /= map.getCellDimension();
        x1 /= map.getCellDimension();
        y0 /= map.getCellDimension();
        y1 /= map.getCellDimension();
        Set<int[]> visitedPoints = new HashSet<>();
        double dx = Math.abs(x1 - x0);
        double dy = Math.abs(y1 - y0);

        int x = (int) (Math.floor(x0));
        int y = (int) (Math.floor(y0));

        double dt_dx = 1.0 / dx;
        double dt_dy = 1.0 / dy;

        double t = 0;

        int n = 1;
        int x_inc, y_inc;
        double t_next_vertical, t_next_horizontal;

        if (dx == 0) {
            x_inc = 0;
            t_next_horizontal = dt_dx; // infinity
        } else if (x1 > x0) {
            x_inc = 1;
            n += (int) (Math.floor(x1)) - x;
            t_next_horizontal = (Math.floor(x0) + 1 - x0) * dt_dx;
        } else {
            x_inc = -1;
            n += x - (int) (Math.floor(x1));
            t_next_horizontal = (x0 - Math.floor(x0)) * dt_dx;
        }

        if (dy == 0) {
            y_inc = 0;
            t_next_vertical = dt_dy; // infinity
        } else if (y1 > y0) {
            y_inc = 1;
            n += (int) (Math.floor(y1)) - y;
            t_next_vertical = (Math.floor(y0) + 1 - y0) * dt_dy;
        } else {
            y_inc = -1;
            n += y - (int) (Math.floor(y1));
            t_next_vertical = (y0 - Math.floor(y0)) * dt_dy;
        }
        if (n == 214748371) {
            return visitedPoints;
        }
        if (n > 10000) {
            return visitedPoints;
        }
        for (; n > 0; --n) {
            visitedPoints.add(new int[]{x, y});

            if (t_next_vertical < t_next_horizontal) {
                y += y_inc;
                t = t_next_vertical;
                t_next_vertical += dt_dy;
            } else {
                x += x_inc;
                t = t_next_horizontal;
                t_next_horizontal += dt_dx;
            }
        }
        return visitedPoints;
    }
}