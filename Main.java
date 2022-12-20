import java.awt.*;
import java.util.ArrayList;
import java.util.Random;

/**
 * Amy Abel, 0007491675, November 22nd 2022
 *
 * 0 Means wall, 1 means walkable node
 **/
public class Main{
    public static void main(String[] args) {
        int wallValue = 0;
        Random rand = new Random();
        //21x21
        int[][] map1 = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                {1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0},
                {0,1,0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,0,0,1,0},
                {0,1,0,1,0,1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,0},
                {0,1,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0},
                {0,1,0,1,0,1,1,1,1,1,0,1,0,1,0,1,1,1,0,1,0},
                {0,1,0,1,0,1,0,1,0,0,0,1,0,1,0,1,0,1,0,0,0},
                {0,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0,1,1,1,0},
                {0,1,0,0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0},
                {0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,1,1,0},
                {0,0,0,1,0,1,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0},
                {0,1,1,1,0,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0},
                {0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0},
                {0,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0},
                {0,1,0,1,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0},
                {0,1,0,1,0,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,0},
                {0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,0,0,1,0},
                {0,1,0,1,1,1,0,1,0,1,0,1,0,1,0,1,1,1,0,1,0},
                {0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,1,0},
                {0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1},
                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

        int[][] map2 = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                {1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0},
                {0,1,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,1,0,1,0},
                {0,1,0,1,1,1,1,1,1,1,0,1,0,1,1,1,0,1,0,1,0},
                {0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0},
                {0,1,0,1,0,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,0},
                {0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0},
                {0,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1,1,1,0,1,0},
                {0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0},
                {0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0},
                {0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,1,0},
                {0,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,0},
                {0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,1,0},
                {0,1,1,1,1,1,1,1,0,1,0,1,0,1,1,1,0,1,1,1,0},
                {0,1,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,1,0},
                {0,1,0,1,1,1,1,1,0,1,1,1,0,1,1,1,0,1,0,1,0},
                {0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0},
                {0,1,0,1,1,1,0,1,0,1,0,1,1,1,1,1,0,1,1,1,0},
                {0,1,0,1,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,1,0},
                {0,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1},
                {0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1},
                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

        // Changing all 1s to to random numbers
        for (int x=0; x<map2.length; x++) {
            for (int y=0; y<map2[x].length; y++) {
                if (map2[x][y] == 1) {
                    map2[x][y] = rand.nextInt(10 - 1) +1;
                }
            }
        }

        //map1
        Pathfinding pathfinding = new Pathfinding(map1, new Point(1,0), new Point(19,20), wallValue);
        ArrayList<Node> solution = pathfinding.AStar(new Euclidian());

        System.out.println("\nSolution:");
        for (Node node : solution) {

            System.out.print("[" + node.coordinates.x + ", " + node.coordinates.y + "], ");
        }

        //map2
        Pathfinding pathfinding2 = new Pathfinding(map2, new Point(1,0), new Point(19,20), wallValue);
        solution = pathfinding2.AStar(new Manhattan());

        System.out.println("\nSolution:");
        for (Node node : solution) {

            System.out.print("[" + node.coordinates.x + ", " + node.coordinates.y + "], ");
        }
    }
}