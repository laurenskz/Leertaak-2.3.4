package threads;

/**
 * Created by Laurens on 17-3-2016.
 */
public class NumberPrintThread extends Thread {

    private int toPrint;
    private Object lock;
    private static int currentToPrint = 4;

    public NumberPrintThread(int toPrint,Object lock) {
        this.toPrint = toPrint;
        this.lock = lock;
    }

    @Override
    public void run() {
        synchronized (lock){
            waitForOthers();
            printNumbers();
            currentToPrint--;
            lock.notifyAll();
        }
    }

    private void printNumbers() {
        for (int i = 0; i < 2; i++) {
            System.out.print(toPrint);
        }
        System.out.println();
    }

    private void waitForOthers() {
        while(toPrint<currentToPrint){
            try {
                lock.wait();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public static void main(String[] args) {
        Object lock = new Object();
        for (int i = 1; i <= 4; i++) {
            new NumberPrintThread(i,lock).start();
        }
    }
}
