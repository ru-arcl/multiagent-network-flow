package projects.multipath.advanced;

import java.lang.reflect.*;
import java.util.*;

/** Access to package-private / private members of the original advanced classes. */
public class CoreAccess {
	public static int rows(Graph g) { return g.rows; }
	public static int cols(Graph g) { return g.columns; }
	public static Graph convert(Graph g, int[] s, int[] t) { return Graph.convertGraph(g, s, t); }
	public static boolean hasPoint(Vertex v) { return v.point != null; }
	public static Problem a9() { return Problem.getA9Puzzle(); }
	public static Graph v4() { return Graph.getv4TestGraph(); }
	public static int[][] SG() { return Graph.SG; }

	static Object call(Object target, Class<?> c, String name, Class<?>[] types, Object... args) throws Throwable {
		Method m = c.getDeclaredMethod(name, types);
		m.setAccessible(true);
		try { return m.invoke(target, args); }
		catch (InvocationTargetException e) { throw e.getCause(); }
	}
	public static boolean connWithout(Graph g, int a, int b, int c) throws Throwable {
		return (Boolean) call(g, Graph.class, "areVerticesConnectedWithoutV3", new Class<?>[]{int.class, int.class, int.class}, a, b, c);
	}
	public static void removeXY(Graph g, int x, int y) throws Throwable {
		call(g, Graph.class, "removeVertex", new Class<?>[]{int.class, int.class}, x, y);
	}
	public static int getId(Graph g, int x, int y) throws Throwable {
		return (Integer) call(g, Graph.class, "getId", new Class<?>[]{int.class, int.class}, x, y);
	}
	public static void writeTxt(Problem p, String fn) throws Throwable {
		call(p, Problem.class, "writeToFile", new Class<?>[]{String.class}, fn);
	}
	public static void setFolder(String f) throws Throwable {
		Field fl = Problem.class.getDeclaredField("FILE_FOLDER");
		fl.setAccessible(true);
		fl.set(null, f);
	}
	public static void family(String name) {
		switch (name) {
		case "n2puzzle": Problem.createN2Puzzles(); break;
		case "crowded": Problem.createCrowdedHardProblems(); break;
		case "32x32": Problem.create32x32Problems(); break;
		case "32x32-8c": Problem.create32x32Problems8Connected(); break;
		case "24x18": Problem.create24x18PerformanceTestingProblems(); break;
		case "crowded-8c": Problem.createCrowdedHardProblems8Connected(); break;
		default: throw new IllegalArgumentException(name);
		}
	}
	/** A graph whose vertices carry points but which has a custom adjacency (for writer crash tests). */
	public static Graph withPoints(int[][] pts, int[][] adj) {
		Graph g = new Graph();
		for (int i = 0; i < pts.length; i++) {
			Vertex v = new Vertex(new java.awt.geom.Point2D.Double(pts[i][0], pts[i][1]));
			v.id = i;
			g.addVertex(v);
			Set<Integer> s = new TreeSet<Integer>();
			for (int u : adj[i]) s.add(u);
			g.adjacencySetMap.put(i, s);
		}
		g.finishBuildingGraph();
		return g;
	}
}
