package model.device;

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


import model.environment.Environment;
import model.environment.Obstacle;
import model.environment.Position;
import model.robot.MobileRobot;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class Laser extends Sensor {


    public Laser(String name, MobileRobot robot, Position localPos, Environment environment) {
        super(name, robot, localPos, environment);

        this.detect = false;
        this.scan = false;


        backgroundColor = Color.cyan;
        this.addPoint(0, 2);
        this.addPoint(100, 2);
        this.addPoint(100, -2);
        this.addPoint(0, -2);
    }

    double read(boolean first) {
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
            if (!obstacle.getOpaque()) {
                double dist = pointToObstacle(obstacle.getPolygon(), centre, front, first);
                if (minDistance == -1.0 || (dist > 0 && dist < minDistance)) {
                    minDistance = dist;
                    if (minDistance > -1 && first) {
                        return minDistance;
                    }
                }
            }
        }
        if (minDistance > 0)
            return minDistance;
        return -1.0;
    }


}
