import java.awt.*;

/**
 * Amy Abel, 0007491675, November 22nd 2022
 * A Node in a map
 * Meant for use with pathfinding
 */
public class Node {
    // The coordinates of the Node
    public Point coordinates;

    // The value of the node
    public double value;

    // The gScore of the node
    public double gScore;

    // The fScore of the node
    public double fScore;


    /**
     * @param x The x coordinate of the Node
     * @param y The y coordinate of the Node
     * @param value The value of the node
     */
    public Node(int x, int y, double value) {
        coordinates = new Point(x, y);
        this.value = value;

        gScore = Double.POSITIVE_INFINITY;
        fScore = Double.POSITIVE_INFINITY;
    }
}
