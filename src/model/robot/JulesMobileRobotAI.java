package model.robot;

import model.virtualmap.OccupancyMap;

import java.io.PipedInputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.PipedOutputStream;
import java.io.IOException;

import java.util.Arrays;
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

public class JulesMobileRobotAI implements Runnable {

	private final OccupancyMap map;
	private final MobileRobot robot;

	private boolean running;

	private double position[] = new double[3];
	private double measures[] = new double[360];

	public JulesMobileRobotAI(MobileRobot robot, OccupancyMap map) {
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

		System.out.println("intelligence running");

		try {
			PipedInputStream pipeIn = new PipedInputStream();
			BufferedReader input = new BufferedReader(new InputStreamReader(pipeIn));
			PrintWriter output = new PrintWriter(new PipedOutputStream(pipeIn), true);

			robot.setOutput(output);
			getPosition(position, input);

			scan(input);
			int[] closestWall = selectClosestWall();

			while (!Arrays.equals(closestWall, new int[]{-1, -1})) {
				System.out.println(Arrays.toString(closestWall));
				pathTo(input, closestWall);
				scan(input);
				map.setFollowed(closestWall[0] / map.getCellDimension(), closestWall[1] / map.getCellDimension());
				closestWall = selectClosestWall();
			}

		} catch (IOException e) {
			System.out.println("Unable to output to Robot");
			e.printStackTrace();
		}
	}

	/**
	 * Selects the closest wall that has not been followed
	 * @return
	 */
	public int[] selectClosestWall() {
		int mapX = (int) (position[0] / map.getCellDimension());
		int mapY = (int) (position[1] / map.getCellDimension());

		int size = 0;

		while (size < Math.max(map.getMapWidth() / map.getCellDimension(), map.getMapHeight() / map.getCellDimension())) {

			for (int x = mapX - (size/2); x < mapX + size; x++) {
				for (int y = mapY - (size/2); y < mapY + size; y++) {
					if (x > map.getMapWidth() || y > map.getMapHeight() || x < 0 || y < 0) {
						continue;
					} else if (map.getGridPoint(x, y,map.getObstacle()) == map.getObstacle() && !map.isFollowed(x, y)) {
						return new int[]{x * map.getCellDimension(), y * map.getCellDimension()};
					}
				}
			}
			
			size++;
		}
		return new int[]{-1, -1};
	}

	/**
	 * Creates a path to the target location
	 *
	 * @param input
	 * @param position double array with the 0'th element the x position and 1st element the y position
	 * @throws IOException
	 */
	private void pathTo(BufferedReader input, int[] position) throws IOException {
		double overstaande = position[0] - this.position[0];
		double aanliggende = position[1] - this.position[1];

		int degrees = (int) Math.toDegrees(Math.atan(overstaande / aanliggende));
		int distance = (int) Math.sqrt(Math.pow(overstaande, 2) + Math.pow(aanliggende, 2));
		System.out.println("Overstaande: " + overstaande + " Aanliggende: " + aanliggende + " Afstand: " + distance + " Hoek: " + degrees);

		rotateTo(input, degrees, this.position);
		moveForward(input, distance);
	}

	/**
	 * Bepaalt of alles wat te ontdekken valt ontdekt is.
	 *
	 * @return
	 */
	private boolean hasMoreToExplore() {
		return true;
	}

	private String moveForward(BufferedReader input, int distance) throws IOException {
		robot.sendCommand("P1.MOVEFW " + distance);
		return input.readLine();
	}

	private void scan(BufferedReader input) throws IOException {
		getPosition(position, input);
		String result;
		robot.sendCommand("L1.SCAN");
		result = input.readLine();
		parseMeasures(result, measures);
		map.drawLaserScan(position, measures);
	}

	private String rotateTo(BufferedReader input, int rotation, double[] position) {
		try {
			getPosition(position, input);

			double toMove = rotation - position[2];
			int degreesAsInt = (int) Math.round(toMove);

			if (degreesAsInt < 0) {
				return rotateLeft(input, -degreesAsInt);
			} else if (degreesAsInt > 0) {
				return rotateRight(input, degreesAsInt);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}

		return null;
	}

	private String rotateLeft(BufferedReader input, int rotation) throws IOException {
		robot.sendCommand("P1.ROTATELEFT " + rotation);
		return input.readLine();
	}

	private String rotateRight(BufferedReader input, int rotation) throws IOException {
		robot.sendCommand("P1.ROTATERIGHT " + rotation);
		return input.readLine();
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
//				System.out.println("direction = " + direction + " distance = " + distance);
			}
		}
	}


}
