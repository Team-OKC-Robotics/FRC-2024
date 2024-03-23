package frc.robot.utils;

import java.util.ArrayList;

public class LerpedLUT {

    private class LUTEntry implements Comparable<LUTEntry>{
        protected double distance;
        protected double angle;

        public LUTEntry(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }

        @Override
        public int compareTo(LUTEntry o) {
            if (o == null) {
                return 1;
            }
            return Double.compare(this.distance, o.distance);
        }
    }

    static ArrayList<LUTEntry> LUT = new ArrayList<>();

    public void addEntry(double distance, double angle) {
        LUT.add(new LUTEntry(distance, angle));
        LUT.sort(null);
    }

    public double getAngleFromDistance(double distance) {

        // Find first index where the distance is larger
        int i = 0;
        for (;i<LUT.size(); i++) {
            if (LUT.get(i).distance >= distance) {
                break;
            }
        }

        // Distance not in our LUT! Just return 25 as a catch all
        if (i >= LUT.size()) {
            return 25;
        }

        // Return the LERP'd value between the nearest two LUT entries
        LUTEntry lower = LUT.get(i - 1);
        LUTEntry higher = LUT.get(i);
        return (lower.angle * (higher.distance - distance) + higher.angle * (distance - lower.distance)) / (higher.distance - lower.distance);
    }
}
