/**
 * @author Vladimir
 *
 */

public class MapNode implements Comparable<MapNode>{
    private GeographicPoint location;
    private double priority;
    private double actualDistance;

    public MapNode(GeographicPoint location){
        this.location = location;
    }

    public GeographicPoint getLocation(){
        GeographicPoint toReturn = location;
        return toReturn;
    }

    public double getActualDistance(){
        return this.actualDistance;
    }

    public void setActualDistance(double distance){
        this.actualDistance = distance;
    }

    public double getPriority(){
        return this.priority;
    }

    public void setPriority(double priority){
        this.priority = priority;
    }

    public int compareTo(MapNode otherMapNode) {
        if (this.priority > otherMapNode.priority) return 1;
        else if (this.priority == otherMapNode.priority) return 0;
        else return -1;
    }

}