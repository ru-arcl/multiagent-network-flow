package projects.multipath.ILP;

import java.lang.reflect.Field;
import java.util.Random;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;

import projects.multipath.advanced.Graph;

/**
 * multiThreadedSplitPlanning loses pieces to its unsynchronized HashMap (original-bugs.md,
 * B14 item 4). Two robots on a 4x6 grid with 20% holes, splits = 2 (the split goals give two
 * one-step pieces, which finish at about the same moment). Usage:
 *
 *   java --add-opens java.base/java.lang=ALL-UNNAMED -cp ORIGINAL_CLASSES:gurobi.jar:. \
 *       projects.multipath.ILP.MtSplitRace 300 hm|chm
 *
 * "hm" runs the original; "chm" replaces the planner's mpath by a ConcurrentHashMap.
 */
public class MtSplitRace {
    static void seedMath(long seed) throws Exception {
        Math.random();
        Class<?> h = Class.forName("java.lang.Math$RandomNumberGeneratorHolder");
        Field f = h.getDeclaredField("randomNumberGenerator");
        f.setAccessible(true);
        ((Random) f.get(null)).setSeed(seed);
    }

    static Graph graph() {
        Graph g = new Graph();
        g.addVertex(0, new int[]{0, 5});
        g.addVertex(1, new int[]{1, 2, 7});
        g.addVertex(2, new int[]{1, 2, 3, 8});
        g.addVertex(3, new int[]{2, 3, 9});
        g.addVertex(4, new int[]{4, 5, 10});
        g.addVertex(5, new int[]{0, 4, 5, 6, 11});
        g.addVertex(6, new int[]{5, 6, 7, 12});
        g.addVertex(7, new int[]{1, 6, 7, 8, 13});
        g.addVertex(8, new int[]{2, 7, 8, 9});
        g.addVertex(9, new int[]{3, 8, 9, 14});
        g.addVertex(10, new int[]{4, 10, 11, 15});
        g.addVertex(11, new int[]{5, 10, 11, 12});
        g.addVertex(12, new int[]{6, 11, 12, 13, 16});
        g.addVertex(13, new int[]{7, 12, 13, 17});
        g.addVertex(14, new int[]{9, 14, 19});
        g.addVertex(15, new int[]{10, 15});
        g.addVertex(16, new int[]{12, 16, 17});
        g.addVertex(17, new int[]{13, 16, 17, 18});
        g.addVertex(18, new int[]{17, 18, 19});
        g.addVertex(19, new int[]{14, 18, 19});
        g.finishBuildingGraph();
        return g;
    }

    public static void main(String[] args) throws Exception {
        int reps = Integer.parseInt(args[0]);
        boolean chm = args.length > 1 && args[1].equals("chm");
        MultiagentGraphSolverGurobiTime.bDebugInfo = false;
        MultiagentGraphSolverGurobiTime.bPrintPaths = false;
        Graph g = graph();
        int nulls = 0;
        TreeSet<String> lost = new TreeSet<String>();
        String plan = null;
        for (int r = 0; r < reps; r++) {
            seedMath(610148160L);
            PathPlanner pp = new PathPlanner();
            if (chm) pp.mpath = new ConcurrentHashMap<Integer, Object>();
            int[][] res = pp.multiThreadedSplitPlanning(g, new int[]{9, 6}, new int[]{1, 2}, 1e6, 2);
            if (res == null) {
                nulls++;
                lost.add(new TreeSet<Integer>(pp.mpath.keySet()).toString());
            } else if (plan == null) {
                plan = java.util.Arrays.deepToString(res);
            }
        }
        System.out.println((chm ? "ConcurrentHashMap" : "HashMap (original)") + ": " + nulls + " of " + reps
                + " runs returned null; pieces kept in those runs: " + lost + "; plan otherwise: " + plan);
        System.exit(0);
    }
}
