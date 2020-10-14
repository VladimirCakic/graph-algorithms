/**
 * @author Vladimir
 *
 */

public class MapEdge {

    private String roadName;
    private String roadType;
    private double length;
    private MapNode destination;

    public MapEdge(MapNode destination, String roadName, String roadType, double length){
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
        this.destination = destination;
    }

    public MapNode getDestination(){
        return this.destination;
    }

    public void setLength(double length){
        this.length = length;
    }

    public double getLength(){
        return length;
    }
}