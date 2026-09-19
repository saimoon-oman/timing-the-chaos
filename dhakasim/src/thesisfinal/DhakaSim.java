package thesisfinal;

import java.util.Date;

public class DhakaSim {
    public static void main(String[] args) {
        // System.out.println("Started at " + new Date());
        Utilities.initialize();
        Utilities.indexTraceFile();
        if (Parameters.GUI_MODE) {
            new DhakaSimFrame();
        } else {
            new Processor().autoProcess();
        }
    }
}
