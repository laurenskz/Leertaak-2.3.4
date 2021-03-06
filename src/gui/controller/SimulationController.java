package gui.controller;

import gui.MainFrame;
import model.robot.MobileRobot;

import javax.swing.*;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

/**
 * Title    :   The Mobile Robot Explorer Simulation Environment v2.0
 * Copyright:   GNU General Public License as published by the Free Software Foundation
 * Company  :   Hanze University of Applied Sciences
 *
 * @author Alexander Jeurissen  (2012)
 * @author Dustin Meijer        (2012)
 * @version 2.0
 */

public class SimulationController extends JMenu implements ActionListener {

	private final JMenuItem menuSimulationStartPause;
	private final JMenuItem menuSimulationReset;

	private final MobileRobot robot;
	private final MainFrame main;

	public SimulationController(MobileRobot robot, MainFrame main) {
		super("Simulation");

		this.robot = robot;
		this.main = main;
		// --------------------------- Menu ----------------------------------------

		// Menu Simulation Start
		this.menuSimulationStartPause = new JMenuItem("Start");
		KeyStroke playPauseKeystroke = KeyStroke.getKeyStroke(KeyEvent.VK_P, 0);
		this.menuSimulationStartPause.setAccelerator(playPauseKeystroke);
		this.menuSimulationStartPause.addActionListener(this);


		// Menu Simulation Reset
		this.menuSimulationReset = new JMenuItem("Reset");
		KeyStroke resetKeystroke = KeyStroke.getKeyStroke(KeyEvent.VK_R, KeyEvent.CTRL_DOWN_MASK);
		this.menuSimulationReset.setAccelerator(resetKeystroke);
		this.menuSimulationReset.addActionListener(this);

		// Menu
		this.add(menuSimulationStartPause);
		this.add(menuSimulationReset);

	}

	private void playPause() {

		if (this.menuSimulationStartPause.getText().equals("Start")) {
			this.robot.start();
			this.menuSimulationStartPause.setText("Pause");
		} else {
			this.robot.quit();
			this.menuSimulationStartPause.setText("Start");
		}
	}



	/**
	 * Invoked when an action occurs.
	 */
	@Override
	public void actionPerformed(ActionEvent e) {

		if (e.getSource().equals(this.menuSimulationStartPause)) {
			playPause();
		} else if (e.getSource().equals(this.menuSimulationReset)) {
			if (this.menuSimulationStartPause.getText().equals("Pause")) {
				this.robot.quit();
				this.menuSimulationStartPause.setText("Start");
				this.main.init();
			} else {
				this.main.init();
			}
		}
	}
}
