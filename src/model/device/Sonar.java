package model.device;

import model.environment.Environment;
import model.environment.Obstacle;
import model.environment.Position;
import model.robot.MobileRobot;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.List;

/**
 * Created by Laurens on 17-3-2016.
 */
public class Sonar extends Sensor {


    List<double[]> circle = new ArrayList<>();

    @Override
    double read(boolean first) {
        draw();
        Point2D centre = new Point2D.Double(localPosition.getX(), localPosition.getY());
        Point2D front = new Point2D.Double(localPosition.getX() + range * Math.cos(localPosition.getT()),
                localPosition.getY() + range * Math.sin(localPosition.getT()));
        // reads the robot's position
        robot.readPosition(robotPosition);
        // center's coordinates according to the robot position
        robotPosition.rotateAroundAxis(centre);
        // front's coordinates according to the robot position
        robotPosition.rotateAroundAxis(front);

        double minDistance = -1.0;
        for (int i = 0; i < environment.getObstacles().size(); i++) {
            // This is really dirty: the laser uses direct access to environment's obstacles
            Obstacle obstacle = environment.getObstacles().get(i);
                double dist = pointToObstacle(obstacle.getPolygon(), centre, front, first);
                if (minDistance == -1.0 || (dist > 0 && dist < minDistance)) {
                    minDistance = dist;
                    if (minDistance > -1 && first) {
                        return minDistance;
                    }
                }
        }
        if (minDistance > 0)
            return minDistance;
        return -1.0;
    }

    private void draw(){
        this.getShape().reset();
        if(this.numSteps==0)return;
        double steps = 360d-this.numSteps;
        steps/= 3.6d;
        for (double[] doubles : circle) {
            this.addPoint((int)(doubles[0]*steps),(int)(doubles[1]*steps));
        }
    }

    public Sonar(String name, MobileRobot robot, Position local, Environment environment){
        super(name, robot, local, environment);
        this.backgroundColor = new Color(255,0,0,125);
        for (int i = 0; i < 360; i++) {
            if(i%15!=0)continue;
            double degrees = Math.toRadians(i);
            circle.add(new double[] {Math.cos(degrees),Math.sin(degrees)});
        }

    }
}
