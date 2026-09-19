package thesisfinal;

import javax.swing.*;
import java.io.*;
import java.util.ArrayList;
import java.util.StringTokenizer;
import java.util.logging.Level;
import java.util.logging.Logger;

import static java.lang.Math.round;

public class DhakaSimFrame extends JFrame {

    public DhakaSimFrame() {
        OptionPanel optionPanel = new OptionPanel(this);
        getContentPane().add(optionPanel);
        setTitle("DhakaSim");
        setSize(1250, 700);
        setExtendedState(JFrame.MAXIMIZED_BOTH);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setVisible(true);
        // Auto-start the simulation once the window is shown (demo recording).
        Timer autoStart = new Timer(800, e -> optionPanel.startSimulation());
        autoStart.setRepeats(false);
        autoStart.start();
    }



}
