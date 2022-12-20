/**
 * Amy Abel, 0007491675, November 22nd 2022
 * Interface for any heuristics we will be using
 **/
public interface Heuristic {
    /**
     * Distance will be set to infinity if goal is a wall
     * @param start The node to start at
     * @param goal The goal
     * @param wallValue The value that walls will be
     * @return The estimated distance between start and goal
     */
    public double EstimateDistance(Node start, Node goal, double wallValue);
}
