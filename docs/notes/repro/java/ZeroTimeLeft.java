package projects.multipath.ILP;
import java.util.*;
import projects.multipath.advanced.*;
/**
 * planHeuristicPath lets timeLeft == 0.0 through (original-bugs.md, B27), and
 * planPathsAdvanced(..., timeLeft) then solves without any time limit. Usage:
 *
 *   java -cp ORIGINAL_CLASSES:gurobi.jar:. projects.multipath.ILP.ZeroTimeLeft 0.5   (returns null)
 *   timeout 25 java -cp ... projects.multipath.ILP.ZeroTimeLeft 0.0                  (never returns)
 */
public class ZeroTimeLeft {
  public static void main(String[] a) throws Exception {
    // budgets of HeuristicPathPlanner.planHeuristicPaths for 6 robots, and the ms elapsed that zero them
    for(int i = 0; i < 3; i ++){ double tl = 1.5 + (5 + i * 4)*6/120.; long ms = Math.round(tl*1000);
      double left = tl - (ms)/1000.; System.out.println("budget " + tl + " elapsed " + ms + " ms -> timeLeft=" + left + " guard(timeLeft<0)=" + (left < 0) + " hasTimeLimit=" + (left > 0)); }
    // the sub-problem the heuristic hands to planPathsAdvanced: path 0-1-6-5-4-3-2
    Graph g = new Graph();
    g.addVertex(0, new int[]{1}); g.addVertex(1, new int[]{0,6}); g.addVertex(6, new int[]{1,5}); g.addVertex(5, new int[]{6,4});
    g.addVertex(4, new int[]{5,3}); g.addVertex(3, new int[]{4,2}); g.addVertex(2, new int[]{3});
    g.finishBuildingGraph();
    int[] s = {4,5,3}, gl = {1,2,0};
    double tl = Double.parseDouble(a[0]);
    MultiagentGraphSolverGurobiTime.bDebugInfo = true; MultiagentGraphSolverGurobiTime.bPrintPaths = false;
    int[][] r = new PathPlanner().planPathsAdvanced(g, s, gl, true, 0, true, tl);
    System.out.println("RESULT tl=" + tl + ": " + (r == null ? "null" : Arrays.deepToString(r)));
    System.exit(0);
  }
}
