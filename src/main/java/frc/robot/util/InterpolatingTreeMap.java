package frc.robot.util;

import java.util.TreeMap;
import java.util.Map.Entry;

public class InterpolatingTreeMap extends TreeMap<Double, Double>{
    public Double get(Double key) {
        Entry<Double, Double> targetEntry = new SimpleEntry<Double, Double>(key, super.get(key));
        if (targetEntry.getValue() != null) return targetEntry.getValue();
        Entry<Double, Double> upperEntry = super.ceilingEntry(key);
        Entry<Double, Double> lowerEntry = super.floorEntry(key);
        if (upperEntry == null && lowerEntry == null) return null;
        else if (upperEntry == null) return lowerEntry.getValue();
        else if (lowerEntry == null) return upperEntry.getValue();
        else return interpolate(targetEntry, upperEntry, lowerEntry);

    }

    public double interpolate(Entry<Double, Double> targetEntry, Entry<Double, Double> upperEntry, Entry<Double, Double> lowerEntry) {
        return (targetEntry.getKey() - lowerEntry.getKey())/(upperEntry.getKey()-lowerEntry.getKey()) *
               (upperEntry.getValue() - lowerEntry.getValue()) + lowerEntry.getValue();

    }
    
}
