/**
 * Amy Abel, 0007491675, November 22nd 2022
 * Uses Manhattan distance to estimate the distance between nodes
 **/
public class Euclidian implements Heuristic {

    /**
     * Distance will be set to infinity if goal is a wall
     * @param start The node to start at
     * @param goal The goal
     * @param wallValue The value that walls will be
     * @return The estimated distance between start and goal using Euclidian distance
     */
    @Override
    public double EstimateDistance(Node start, Node goal, double wallValue) {
        if (goal.value == wallValue) {
            return Double.POSITIVE_INFINITY;
        }
        else {
            return ( (Math.sqrt( ((goal.coordinates.x - start.coordinates.x) * (goal.coordinates.x - start.coordinates.x)) + ( (goal.coordinates.y - start.coordinates.y) * (goal.coordinates.y - start.coordinates.y) ) *goal.value )));
        }

    }
}
