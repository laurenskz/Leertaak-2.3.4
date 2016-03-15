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

    public static final int MAX_WALL_DISTANCE = 70;
    public static final int MAX_BOX_DIFFERENCE = 8;
    public static final int MOVE_DISTANCE = 10;
    private final OccupancyMap map;
    private final MobileRobot robot;
    private double robotX, robotY, robotWidth, robotHeight;
    private int[] lastFollowedWallBlock;

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


    private void mazeWallFollower(double[] position, double[] measures, BufferedReader input) throws IOException{
        getPosition(position,input);
        int degrees = (int)Math.round(position[2]);
        scan(position,measures,input);
        int stepsSinceRight = Integer.MIN_VALUE;
        while(true){
            while(safeMove(position, MOVE_DISTANCE)){
                getPosition(position,input);
                move(input,MOVE_DISTANCE);
                stepsSinceRight++;
                if(stepsSinceRight>2&&canGoRight(position,input)){
                    degrees = (degrees+90)%360;
                    rotateTo(degrees,input,position);
                    getPosition(position, input);
                    stepsSinceRight = 0;

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
    private void testFollowWallPoint(double[] position, double[] measures, BufferedReader input) throws IOException {
        getPosition(position, input);
        scan(position, measures, input);
        map.getGrid()[15][19] = OccupancyMap.TOFOLLOW;
        map.processEvent(new ActionEvent(map, 0, ""));
        int[] ret = followWallPoint(new int[]{15, 19}, position);
        rotateTo(ret[0], input, position);
        move(input, ret[1]);
    }

    private void loop(double[] position, double[] measures, BufferedReader input) throws IOException {
        while (get()) {
            System.out.println("start of loop");
            getPosition(position, input);
            scan(position, measures, input);
            List<int[]> wallBlocks = getWallBlocksToFollow(position);
            System.out.println("wallblock size = " + wallBlocks.size());
            if (wallBlocks.size() == 0) {
                continue;
            }
            getPosition(position, input);
            double[] startPos = clone(position);
            for (int[] wallBlock : wallBlocks) {
                System.out.println(Arrays.toString(wallBlock));
                int[] wallPoint = followWallPoint(wallBlock, position);
                System.out.println(Arrays.toString(wallPoint));
                if (wallPoint == null) continue;
                map.getGrid()[wallBlock[0]][wallBlock[1]] = OccupancyMap.TOFOLLOW;
                map.processEvent(new ActionEvent(map, 0, ""));
                System.out.println("Going to the wallBlock : " + Arrays.toString(wallBlock));
                System.out.println("Going to the wallPoint : " + Arrays.toString(wallPoint));
                rotateTo(wallPoint[0], input, position);
                move(input, wallPoint[1]);
                map.getGrid()[wallBlock[0]][wallBlock[1]] = map.getObstacle();
                map.processEvent(new ActionEvent(map, 0, ""));
                lastFollowedWallBlock = wallBlock;
                break;
            }
            getPosition(position, input);
            double degrees = Math.toRadians(position[2]);
            double xSurplus = Math.cos(degrees), ySurplus = Math.sin(degrees);
            Set<int[]> visitedSquares = raytrace(startPos[0], startPos[1], position[0] + xSurplus, position[1] + ySurplus, map);
            for (int[] block : wallBlocks) {
                if (visited(block, visitedSquares)) {
                    map.setFollowed(block[0], block[1]);
                }
            }
        }
    }

    private double[] clone(double[] ar) {
        if (ar == null) return null;
        double[] toRet = new double[ar.length];
        for (int i = 0; i < ar.length; i++) {
            toRet[i] = ar[i];
        }
        return toRet;
    }

    private boolean visited(int[] wallPoint, Set<int[]> visitedSquares) {
        boolean followableX, followableY;
        followableX = map.getGridPoint(wallPoint[0] - 1, wallPoint[1], map.getObstacle()) == map.getEmpty() || map.getGridPoint(wallPoint[0] + 1, wallPoint[1], map.getObstacle()) == map.getEmpty();
        followableY = map.getGridPoint(wallPoint[0], wallPoint[1] - 1, map.getObstacle()) == map.getEmpty() || map.getGridPoint(wallPoint[0], wallPoint[1] + 1, map.getObstacle()) == map.getEmpty();
        for (int[] visitedSquare : visitedSquares) {
            int difference = Math.abs(wallPoint[0] - visitedSquare[0]) + Math.abs(wallPoint[1] - visitedSquare[1]);
            if (followableY)
                if (visitedSquare[0] == wallPoint[0]) return difference < MAX_BOX_DIFFERENCE;
            if (followableX)
                if (visitedSquare[1] == wallPoint[1]) return difference < MAX_BOX_DIFFERENCE;
        }
        return false;
    }

    private int[] followWallPoint(int[] wallPoint, double[] position) {
        boolean followableX, followableY;
        followableX = map.getGridPoint(wallPoint[0] - 1, wallPoint[1], map.getObstacle()) == map.getEmpty() || map.getGridPoint(wallPoint[0] + 1, wallPoint[1], map.getObstacle()) == map.getEmpty();
        followableY = map.getGridPoint(wallPoint[0], wallPoint[1] - 1, map.getObstacle()) == map.getEmpty() || map.getGridPoint(wallPoint[0], wallPoint[1] + 1, map.getObstacle()) == map.getEmpty();
        if (!followableX && !followableY) return null;
        double dY = wallPoint[1] * map.getCellDimension() - position[1];
        double dX = wallPoint[0] * map.getCellDimension() - position[0];
        double[] newPosition = new double[3];
        for (int i = 0; i < 3; i++) {
            newPosition[i] = position[i];
        }
        int currentDegrees = (int) position[2];
        for (int i = 0; i < 180; i++) {
            double degrees = Math.toRadians(((currentDegrees + i) % 360));
            int[] distance = calulateAngle(wallPoint, followableX, followableY, dY, dX, newPosition, ((currentDegrees + i) % 360), degrees);
            if (distance != null) return distance;
            degrees = Math.toRadians(((currentDegrees - i) % 360));
            distance = calulateAngle(wallPoint, followableX, followableY, dY, dX, newPosition, ((currentDegrees - i) % 360), degrees);
            if (distance != null) return distance;
        }
        return null;
    }

    private int[] calulateAngle(int[] wallPoint, boolean followableX, boolean followableY, double dY, double dX, double[] newPosition, int angle, double degrees) {
        double sin = Math.sin(degrees), cos = Math.cos(degrees);
        double distance = 0;
        if (followableY) {
            if (Math.abs(cos) < 0.00001) return null;
            distance = dX / cos;
        }
        if (distance > 0) {
            newPosition[2] = angle;
            double[] endPoint = translateWithAngleAndDistance(newPosition, (int) distance);
            if (Math.abs(endPoint[1] - wallPoint[1] * map.getCellDimension()) > MAX_WALL_DISTANCE) return null;
            if (safeMove(newPosition, (int) distance)) return new int[]{angle, (int) distance};
        }
        if (followableX) {
            if (Math.abs(sin) < 0.00001) return null;
            distance = dY / sin;
        }
        if (distance <= 0) return null;
        newPosition[2] = angle;
        double[] endPoint = translateWithAngleAndDistance(newPosition, (int) distance);
        if (Math.abs(endPoint[0] - wallPoint[0] * map.getCellDimension()) > MAX_WALL_DISTANCE) return null;
        if (safeMove(newPosition, (int) Math.round(distance))) return new int[]{angle, (int) Math.round(distance)};
        return null;
    }

    private boolean safeMove(double[] position, int distance) {
        double[] positionClone = clone(position);
        positionClone[2] = Math.round(position[2]);
        Set<int[]> pointsOnRoute = pointsBetween(positionClone, distance);
//        pointsOnRoute.addAll(getTurnPoints(position, distance));
        for (int[] ints : pointsOnRoute) {
            try {
                if(map.getGrid()[ints[0]][ints[1]]==map.getEmpty())
                    map.getGrid()[ints[0]][ints[1]] = OccupancyMap.TOFOLLOW;
            } catch (Exception e) {
//                e.printStackTrace();
            }
        }
        map.processEvent(new ActionEvent(map,0,""));
        try {
            Thread.sleep(0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        for (int[] ints : pointsOnRoute) {
            try {
                if(map.getGrid()[ints[0]][ints[1]]==OccupancyMap.TOFOLLOW)
                    map.getGrid()[ints[0]][ints[1]] = map.getEmpty();
            } catch (Exception e) {
//                e.printStackTrace();
            }
        }
        for (int[] ints : pointsOnRoute) {
            if (map.getGridPoint(ints[0], ints[1], map.getObstacle()) != map.getEmpty() && map.getGridPoint(ints[0], ints[1], map.getObstacle()) != map.getRobot()) {
                return false;
            }
        }
        return true;
    }

    private Set<int[]> getTurnPoints(double[] position, int distance) {
        Set<int[]> toReturn = new HashSet<>();
        double[] circleCenter = translateWithAngleAndDistance(position, distance);
        double robotMaxHeight = robotHeight + robotX;
        double robotMaxWidth = robotWidth + robotY;
        double radius = Math.sqrt(robotMaxHeight * robotMaxHeight + robotMaxWidth * robotMaxWidth);
        for (int i = 0; i < 9; i++) {
            circleCenter[2] = (i * (360d / 9)) + 1;
            double[] circleEdge = translateWithAngleAndDistance(circleCenter, (int) radius + 1);
            toReturn.addAll(raytrace(circleCenter[0], circleCenter[1], circleEdge[0], circleEdge[1], map));
        }
        return toReturn;
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
        int ySize = map.getMapHeight() / map.getCellDimension();
        int[][] xValues = new int[ySize][2];
        for (int[] xValue : xValues) {
            for (int i = 0; i < xValue.length; i++) {
                xValue[i] = -1;
            }
        }
        for (int[] point : visitedPoints) {
            if (point[1] < 0 || point[1] >= xValues.length) continue;
            if (point[0] == -1) continue;
            if (xValues[point[1]][0] == -1) xValues[point[1]][0] = point[0];
            if (xValues[point[1]][0] > point[0]) xValues[point[1]][0] = point[0];
            if (xValues[point[1]][1] < point[0]) xValues[point[1]][1] = point[0];
        }
        for (int i = 0; i < xValues.length; i++) {
            int[] xMinAndMax = xValues[i];
            if (xMinAndMax[0] < 0 || xMinAndMax[1] < 0) continue;
            for (int j = xMinAndMax[0]; j < xMinAndMax[1]; j++) {
                visitedPoints.add(new int[]{j, i});
            }
        }
        return visitedPoints;
    }

    private double[][] getRobotBounds(double[] position) {
        double vx = robotWidth + robotY;
        double vy = robotHeight / 2;
        vx+=5;//Safety!!!!!!
        vy+=0;
        double degrees = Math.toRadians(360d - position[2]);
        double[] leftFront = new double[2];
        leftFront[0] = position[0] + vx * Math.cos(degrees) - vy * Math.sin(degrees);
        leftFront[1] = position[1] - vx * Math.sin(degrees) - vy * Math.cos(degrees);
        double[] rightFront = new double[2];
        rightFront[0] = leftFront[0] + (2 * vy) * Math.sin(degrees);
        rightFront[1] = leftFront[1] + (2 * vy) * Math.cos(degrees);
        double[] leftBack = new double[2];
        leftBack[0] = leftFront[0] - robotWidth * Math.cos(degrees);
        leftBack[1] = leftFront[1] + robotWidth * Math.sin(degrees);
        double[] rightBack = new double[2];
        rightBack[0] = leftBack[0] + (2 * vy) * Math.sin(degrees);
        rightBack[1] = leftBack[1] + (2 * vy) * Math.cos(degrees);
        return new double[][]{leftFront, rightFront, leftBack, rightBack};
    }

    private void rotateTo(double degrees, BufferedReader input, double[] position) {
        try {
            getPosition(position, input);
            double toMove = degrees - position[2];
            int degreesAsInt = (int) Math.round(toMove);
            if(degreesAsInt>180){
                degreesAsInt-=360;
            }
            if(degreesAsInt<-180){
                degreesAsInt+=360;
            }
            if (degreesAsInt < 0) {
                rotateLeft(input, -degreesAsInt);
            } else if (degreesAsInt > 0) {
                rotateRight(input, degreesAsInt);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void moveTo(double[] position, double[] currentPosition, BufferedReader input) {
        try {
            getPosition(currentPosition, input);
        } catch (IOException e) {
            e.printStackTrace();
        }
        double dX = position[0] - currentPosition[0];//Aanliggende zijde
        double dY = position[1] - currentPosition[1];//Overstaande zijde
        double degrees = getDegrees(dX, dY);
        moveRobot(currentPosition, input, dX, dY, degrees);
    }

    private void moveRobot(double[] currentPosition, BufferedReader input, double dX, double dY, double degrees) {
        rotateTo(degrees, input, currentPosition);
        int distance = (int) Math.sqrt(dX * dX + dY * dY);
        move(input, distance);
    }

    private static double getDegrees(double dX, double dY) {
        double tan = dY / dX;
        double radians = Math.atan(tan);
        double degrees = Math.toDegrees(radians);
        if (dX < 0)
            degrees += 180d;
        else if (dY < 0)
            degrees += 360d;
        return degrees;
    }

    private List<int[]> getWallBlocksToFollow(double[] position) {
        List<int[]> possibilities = new ArrayList<>();
        for (int i = 0; i < map.getGrid().length; i++) {
            for (int j = 0; j < map.getGrid()[i].length; j++) {
                if (map.getGrid()[i][j] == map.getObstacle()) {
                    if (!map.isFollowed(i, j)) {
                        possibilities.add(new int[]{i, j});
                    }
                }
            }
        }
        return orderWallBlocks(possibilities, position);
    }

    private List<int[]> orderWallBlocks(List<int[]> wallBlocks, double[] position) {
        wallBlocks.sort(new Comparator<int[]>() {
            @Override
            public int compare(int[] o1, int[] o2) {
                return distance(o2, position) - distance(o1, position);
            }
        });
        return wallBlocks;
    }

    private int distance(int[] wallBlock, double[] position) {
        if(lastFollowedWallBlock==null)return 0;
        if(lastFollowedWallBlock[0]==wallBlock[0])return 1000;
        if(lastFollowedWallBlock[1]==wallBlock[1])return 1000;
        double dX = Math.abs(wallBlock[0] * map.getCellDimension() - lastFollowedWallBlock[0] * map.getCellDimension());
        double dY = Math.abs(wallBlock[1] * map.getCellDimension() - lastFollowedWallBlock[1] * map.getCellDimension());
        return Integer.MAX_VALUE-((int)Math.sqrt(dX*dX+dY*dY));
    }


    private boolean get() {
        return true;
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
//        int[][] deltas = new int[][]{
//                new int[]{-1,-1},
//                new int[]{-2,-1},
//                new int[]{-1,0},
//                new int[]{-1,1},
//                new int[]{-1,2},
//                new int[]{0,1},
//                new int[]{1,1},
//                new int[]{2,1},
//                new int[]{1,0},
//                new int[]{2,-1},
//                new int[]{1,-1},
//                new int[]{0,-1},
//        };
//        for (int i = 0; i < deltas.length; i++) {
//            System.out.printf("dX = %d dY = %d\n",deltas[i][0],deltas[i][1]);
//            System.out.printf("degrees = %f\n",getDegrees(deltas[i][0],deltas[i][1]));
//        }

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
