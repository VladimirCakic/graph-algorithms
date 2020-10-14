/**
 * @author Vladimir
 *
 */
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;


public class MapGraph {

    private int numOfVertices;
    private int numOfEdges;
    private HashMap<MapNode, List<MapEdge>> map;
    private int nOfVisited;
    private List<GeographicPoint> visitedNodes;

    public MapGraph() {
        map = new HashMap<MapNode, List<MapEdge>>();
        numOfVertices = 0;
        numOfEdges = 0;
    }

    public int getNumVertices() {
        return numOfVertices;
    }

    public Set<GeographicPoint> getVertices() {
        Set<GeographicPoint> toReturn = new HashSet<GeographicPoint>();
        for (MapNode node: map.keySet()){
            toReturn.add(node.getLocation());
        }
        return toReturn;
    }

    public int getNumEdges() {
        return numOfEdges;
    }

    public boolean addVertex(GeographicPoint location) {
        if (location == null || getVertices().contains(location))
            return false;
        else{
            map.put(new MapNode(location), new LinkedList<MapEdge>());
            numOfVertices++;
            return true;
        }
    }

    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {

        if (length<0 || roadType == null || roadName == null
                || !getVertices().contains(to) || !getVertices().contains(from)){
            throw new IllegalArgumentException();
        }else{
            if (!getNeighbors(getNodeAtLocation(from)).contains(getNodeAtLocation(to))){
                map.get(getNodeAtLocation(from)).add(new MapEdge(getNodeAtLocation(to), roadName, roadType, length));
                numOfEdges++;
            }

        }

    }

    private MapNode getNodeAtLocation(GeographicPoint location){
        for (MapNode node: map.keySet()){
            if (node.getLocation().equals(location)) return node;
        }
        return null;
    }

    private List<MapNode> getNeighbors(MapNode node){
        List<MapNode> toReturn = new LinkedList<MapNode>();
        List<MapEdge> roads = map.get(node);
        for (MapEdge road: roads){
            toReturn.add(road.getDestination());
        }
        return toReturn;
    }

    private MapEdge getEdgeBetweenTwoNodes(MapNode currNode, MapNode destinationNode){
        List<MapEdge> edges = map.get(currNode);
        for (MapEdge edge: edges){
            if (destinationNode.equals(edge.getDestination())){
                return edge;
            }
        }
        return null;
    }


    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {

        if (!map.containsKey(getNodeAtLocation(start)) || !map.containsKey(getNodeAtLocation(goal))){
            System.out.println("Start or goal node is null!  No path exists.");
            return new LinkedList<GeographicPoint>();
        }
        nOfVisited=0;
        HashSet<MapNode> visited = new HashSet<MapNode>();
        Queue<MapNode> toExplore = new LinkedList<MapNode>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        toExplore.add(getNodeAtLocation(start));
        boolean found = false;
        while (!toExplore.isEmpty()) {
            MapNode curr = toExplore.remove(); nOfVisited++;
            if (curr.equals(getNodeAtLocation(goal))) {
                found = true;
                break;
            }
            List<MapNode> neighbors = getNeighbors(curr);
            for (MapNode next: neighbors){
                if (!visited.contains(next)) {
                    visited.add(next);
                    parentMap.put(next.getLocation(), curr.getLocation());
                    toExplore.add(next);
                }
            }
        }

        if (!found) {
            System.out.println("No path exists");
            return null;
        }


        // reconstruct the path
        LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
        GeographicPoint curr = goal;
        while (!curr.equals(start)) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        path.addFirst(start);
        return path;
    }


    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal) {

        if (!map.containsKey(getNodeAtLocation(start)) || !map.containsKey(getNodeAtLocation(goal))){
            System.out.println("Start or goal node is null!  No path exists.");
            return new LinkedList<GeographicPoint>();
        }

        for (MapNode node: map.keySet()){
            node.setPriority(Double.POSITIVE_INFINITY);
        }
        HashSet<MapNode> visited = new HashSet<MapNode>();
        PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        nOfVisited=0;
        visitedNodes = new LinkedList<GeographicPoint>();

        /*Do the search*/
        getNodeAtLocation(start).setPriority(0);
        toExplore.offer(getNodeAtLocation(start));
        boolean found = false;
        while (!toExplore.isEmpty()) {
            MapNode curr = toExplore.poll(); nOfVisited++; visitedNodes.add(curr.getLocation());
            if (!visited.contains(curr)){
                visited.add(curr);
                if (curr.equals(getNodeAtLocation(goal))) {
                    found = true;
                    break;
                }
                List<MapNode> neighbors = getNeighbors(curr);
                for (MapNode next: neighbors){
                    if (!visited.contains(next)){
                        if((curr.getPriority() + getEdgeBetweenTwoNodes(curr,next).getLength()) < next.getPriority()){
                            next.setPriority(curr.getPriority() + getEdgeBetweenTwoNodes(curr,next).getLength());
                            parentMap.put(next.getLocation(), curr.getLocation());
                            toExplore.offer(next);
                        }
                    }
                }
            }

        }

        if (!found) {
            System.out.println("No path exists");
            return null;
        }

        // reconstruct the path
        LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
        GeographicPoint curr = goal;
        while (!curr.equals(start)) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        path.addFirst(start);
        return path;
    }


    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {

        if (!map.containsKey(getNodeAtLocation(start)) || !map.containsKey(getNodeAtLocation(goal))){
            System.out.println("Start or goal node is null!  No path exists.");
            return new LinkedList<GeographicPoint>();
        }

        for (MapNode node: map.keySet()){
            node.setPriority(Double.POSITIVE_INFINITY);
            node.setActualDistance(Double.POSITIVE_INFINITY);
        }
        HashSet<MapNode> visited = new HashSet<MapNode>();
        PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
        HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<GeographicPoint, GeographicPoint>();
        nOfVisited=0;
        visitedNodes = new LinkedList<GeographicPoint>();

        /*Do the search*/
        getNodeAtLocation(start).setPriority(0);
        getNodeAtLocation(start).setActualDistance(0);
        toExplore.add(getNodeAtLocation(start));
        boolean found = false;
        while (!toExplore.isEmpty()) {
            MapNode curr = toExplore.remove(); nOfVisited++; visitedNodes.add(curr.getLocation());
            if (!visited.contains(curr)){
                visited.add(curr);
                if (curr.equals(getNodeAtLocation(goal))) {
                    found = true;
                    break;
                }
                List<MapNode> neighbors = getNeighbors(curr);
                for (MapNode next: neighbors){
                    if (!visited.contains(next)){
                        if((curr.getActualDistance() + getEdgeBetweenTwoNodes(curr,next).getLength()
                                + next.getLocation().distance(goal)) < next.getPriority()){
                            next.setPriority(curr.getActualDistance()
                                    + getEdgeBetweenTwoNodes(curr,next).getLength()
                                    + next.getLocation().distance(goal));
                            next.setActualDistance(curr.getActualDistance()
                                    + getEdgeBetweenTwoNodes(curr,next).getLength());
                            parentMap.put(next.getLocation(), curr.getLocation());
                            toExplore.add(next);
                        }
                    }
                }
            }



        }

        if (!found) {
            System.out.println("No path exists");
            return null;
        }


        // reconstruct the path
        LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
        GeographicPoint curr = goal;
        while (!curr.equals(start)) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        path.addFirst(start);
        return path;
    }

    public static void printPath(List<GeographicPoint> path){
        for (GeographicPoint point: path){
            System.out.println(point + "     ");
        }
    }



    public static void main(String[] args)
    {

		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("src/main/resources/simpletest.map", firstMap);
		System.out.println("DONE.");


		System.out.println(firstMap.numOfVertices);
		System.out.println(firstMap.numOfEdges);

		for (MapNode node: firstMap.map.keySet()){
			List<MapNode> neighbors = firstMap.getNeighbors(node);
			System.out.println(node.getLocation() + "  ->  ");
			System.out.println();
			for (MapNode neighbor: neighbors){
				System.out.println(neighbor.getLocation());
			}

		}

		GeographicPoint start = new GeographicPoint(1.0, 1.0);
		GeographicPoint goal = new GeographicPoint(8.0, -1.0);
		List<GeographicPoint> path = firstMap.bfs(start, goal);
		for (GeographicPoint point: path){
			System.out.print(point + "  ->  ");
		}



		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("src/main/resources/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		printPath(testroute);
		System.out.println(simpleTestMap.nOfVisited);
		System.out.println("Nodes visited: ");
		printPath(simpleTestMap.visitedNodes);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		printPath(testroute2);
		System.out.println(simpleTestMap.nOfVisited);
		System.out.println("Nodes visited: ");
		printPath(simpleTestMap.visitedNodes);


		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("src/main/resources/utc.map", testMap);

		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		printPath(testroute);
		System.out.println(testMap.nOfVisited);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		printPath(testroute2);
		System.out.println(testMap.nOfVisited);


		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		printPath(testroute);
		System.out.println(testMap.nOfVisited);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		printPath(testroute2);
		System.out.println(testMap.nOfVisited);




        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("src/main/resources/utc.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start1 = new GeographicPoint(32.8648772, -117.2254046);
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

        List<GeographicPoint> route0 = theMap.bfs(start1,end);
        System.out.println(theMap.nOfVisited);
        List<GeographicPoint> route1 = theMap.dijkstra(start1,end);
        System.out.println(theMap.nOfVisited);
        List<GeographicPoint> route2 = theMap.aStarSearch(start1,end);
        System.out.println(theMap.nOfVisited);

    }

}

