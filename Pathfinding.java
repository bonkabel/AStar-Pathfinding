import java.awt.*;
import java.lang.reflect.Array;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

/**
 * Amy Abel, 0007491675, November 22nd 2022
 *
 * AStar pathfinding algorithm 
 */
public class Pathfinding {

    /**
     * The int that walls will be in the input intMap
     */
    int wallInput = 0;

    /**
     * The value that
     */
    final double WALLVALUE = Double.POSITIVE_INFINITY;

    /**
     * I we output debug information
     */
    final boolean DEBUG = true;

    /**
     *  The map of Nodes to navigate
     */
    public Node[][] map;

    /**
     * The goal in the map
     */
    private Node goal;

    /**
     * The start in the map
     */
    private Node start;

    /**
     *
     * @param intMap a 2d array of ints that represents a map
     *               wallInput = wall
     *               anything else is walkable
     * @param start The start of the maze
     * @param goal The goal of the maze
     * @param wallInput The int value that walls will be in intMap
     */
    public Pathfinding(int[][] intMap, Point start, Point goal, int wallInput) {
        this.wallInput = wallInput;

        // Setuping up the map
        map = new Node[intMap.length][intMap[0].length];
        for (int i=0; i<intMap.length; i++) {
            for (int j=0; j<intMap[i].length; j++) {
                if (intMap[i][j] == wallInput) {
                    map[i][j] = new Node(i,j, WALLVALUE);
                }
                else {
                    map[i][j] = new Node(i,j, intMap[i][j]);
                }

            }
        }

        this.start = map[start.x][start.y];
        this.goal = map[goal.x][goal.y];


    }

    /**
     * Implementation of the A* algorithm
     * @param heuristic The heuristic to use
     * @return The solution to get from start to goal
     */
    public ArrayList<Node> AStar(Heuristic heuristic) {
        ArrayList<Node> openSet = GetOpenSet(start);
        HashMap<Node, Node> cameFrom = new HashMap<>();
        start.gScore = 0;
        start.fScore = heuristic.EstimateDistance(start, goal, WALLVALUE);


        while (!openSet.isEmpty()) {
            Node current = GetLowestfScore(openSet);

            if (DEBUG) {
                System.out.println("\nThe current node is: [" + current.coordinates.x + ", " + current.coordinates.y + "] with an fScore of " + new DecimalFormat("#.##").format(current.fScore));
                System.out.println("The openSet is:");
                for (Node node : openSet) {
                    System.out.println("\tNode: [" + node.coordinates.x + ", " + node.coordinates.y + "] with an fScore of " + new DecimalFormat("#.##").format(node.fScore));
                }
            }

            if (current == goal) {
                return ReconstructPath(cameFrom, current);
            }

            openSet.remove(current);
            if (DEBUG) {
                System.out.println("The neighbors are:");
            }

            for (Node neighbor : GetNeighbors(current)) {
                double tentativegScore = current.gScore + heuristic.EstimateDistance(current, neighbor, WALLVALUE);
                if (tentativegScore < neighbor.gScore) {
                    cameFrom.put(neighbor, current);
                    neighbor.gScore = tentativegScore;
                    neighbor.fScore = neighbor.gScore + heuristic.EstimateDistance(neighbor, goal, WALLVALUE);

                    if (!openSet.contains(neighbor)) {
                        openSet.add(neighbor);
                    }
                }

                if (DEBUG) {
                    System.out.println("\tNode: [" + neighbor.coordinates.x + ", " + neighbor.coordinates.y + "] with an fScore of " + new DecimalFormat("#.##").format(neighbor.fScore));
                }
            }
        }


        return null;
    }


    /**
     * Gets the neighbors
     * @param current The current node
     * @return The neighbors
     */
    private ArrayList<Node> GetNeighbors(Node current) {
        ArrayList<Node> neighbors = new ArrayList<>();

        for (int x=current.coordinates.x-1; x<=current.coordinates.x+1; x++) {
            for (int y=current.coordinates.y-1; y<=current.coordinates.y+1; y++) {
                if ( insideMap(x,y) && !(x == current.coordinates.x && y == current.coordinates.y)) {
                    neighbors.add(map[x][y]);
                }
            }
        }

        return neighbors;
    }

    /**
     * Gets the openSet
     * @param start The starting node
     * @return The openset
     */
    private ArrayList<Node> GetOpenSet(Node start) {
        ArrayList<Node> openSet = new ArrayList<>();

        for (int x=start.coordinates.x-1; x<=start.coordinates.x+1; x++) {
            for (int y=start.coordinates.y-1; y<=start.coordinates.y+1; y++) {
                if ( insideMap(x,y) ) {
                    openSet.add(map[x][y]);
                }
            }
        }

        return openSet;
    }

    // Checks to see if a coordinate is inside the map
    private boolean insideMap(int x, int y) {
        boolean inside = true;
        if (x < 0 || y < 0) {
            inside = false;
        }
        else if (x > map.length -1 || y > map[x].length - 1) {
            inside = false;
        }

        return inside;
    }

    /**
     * Finds the lowest fscore in the openset
     * @param openSet the openset to find the lowest fscore in
     * @return the Node with the lowest fscore
     */
    private Node GetLowestfScore(ArrayList<Node> openSet) {
        Node lowestfScore = null;

        for (int i=0; i<openSet.size(); i++) {
            if (lowestfScore == null || openSet.get(i).fScore < lowestfScore.fScore) {
                lowestfScore = openSet.get(i);
            }
        }

        return lowestfScore;
    }

    /**
     * @param cameFrom A HashMap of where each node came from
     * @param current  The current node
     * @return  A reconstruction of the path that was taken to get to the goal
     */
    private ArrayList<Node> ReconstructPath(HashMap<Node, Node> cameFrom, Node current) {
         ArrayList<Node> path = new ArrayList<>();
         path.add(current);

         while (cameFrom.containsKey(current)) {
             //System.out.println("Current: [" + current.coordinates.x + ", " + current.coordinates.y + "]");
             current = cameFrom.get(current);
             path.add(current);
         }

         //Reverse the path so it's in order
         Collections.reverse(path);

         return path;
    }
}
