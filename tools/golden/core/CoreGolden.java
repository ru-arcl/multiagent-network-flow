package projects.multipath.ILP;

import java.io.*;
import java.lang.reflect.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.DirectoryStream;
import java.security.MessageDigest;
import java.util.*;

import projects.multipath.advanced.*;
import projects.multipath.advanced.Path;

/**
 * Recorder of tests/data/core/{graph,problem,pathfinder,families}.json for mppflow core
 * (graph.py, problem.py, pathfinder.py); run by tools/golden/core/record_core.sh.
 * Usage: CoreGolden <outdir> <tmpdir> [graph,problem,pathfinder,families]
 */
public class CoreGolden {
	static void seed(long s) throws Exception {
		Class<?> c = Class.forName("java.lang.Math$RandomNumberGeneratorHolder");
		Field f = c.getDeclaredField("randomNumberGenerator");
		f.setAccessible(true);
		((Random) f.get(null)).setSeed(s);
	}

	// ------------------------------------------------------------ tiny JSON
	static String q(String s) {
		StringBuilder sb = new StringBuilder("\"");
		for (char ch : s.toCharArray()) {
			switch (ch) {
			case '"': sb.append("\\\""); break;
			case '\\': sb.append("\\\\"); break;
			case '\n': sb.append("\\n"); break;
			case '\r': sb.append("\\r"); break;
			case '\t': sb.append("\\t"); break;
			default:
				if (ch < 0x20) sb.append(String.format("\\u%04x", (int) ch)); else sb.append(ch);
			}
		}
		return sb.append('"').toString();
	}
	static String obj(Object... kv) {
		StringBuilder sb = new StringBuilder("{");
		for (int i = 0; i < kv.length; i += 2) {
			if (i > 0) sb.append(", ");
			sb.append(q((String) kv[i])).append(": ").append(val(kv[i + 1]));
		}
		return sb.append("}").toString();
	}
	static String val(Object o) {
		if (o == null) return "null";
		if (o instanceof Raw) return ((Raw) o).s;
		if (o instanceof String) return q((String) o);
		if (o instanceof Integer || o instanceof Long || o instanceof Boolean) return o.toString();
		if (o instanceof Double) return Double.toString((Double) o);
		if (o instanceof int[]) { int[] a = (int[]) o; StringBuilder sb = new StringBuilder("["); for (int i = 0; i < a.length; i++) { if (i > 0) sb.append(", "); sb.append(a[i]); } return sb.append("]").toString(); }
		if (o instanceof int[][]) { int[][] a = (int[][]) o; StringBuilder sb = new StringBuilder("["); for (int i = 0; i < a.length; i++) { if (i > 0) sb.append(", "); sb.append(val(a[i])); } return sb.append("]").toString(); }
		if (o instanceof Collection) { StringBuilder sb = new StringBuilder("["); int i = 0; for (Object x : (Collection<?>) o) { if (i++ > 0) sb.append(", "); sb.append(val(x)); } return sb.append("]").toString(); }
		throw new IllegalArgumentException(o.getClass().toString());
	}
	static class Raw { String s; Raw(String s) { this.s = s; } }
	static Raw raw(String s) { return new Raw(s); }

	static String hex(byte[] b) { StringBuilder sb = new StringBuilder(); for (byte x : b) sb.append(String.format("%02x", x & 0xff)); return sb.toString(); }
	static String sha(byte[] b) throws Exception { return hex(MessageDigest.getInstance("SHA-256").digest(b)); }

	static Raw dump(Graph g) {
		List<Integer> vs = new ArrayList<>();
		for (Vertex v : g.vertices) vs.add(v.id);
		StringBuilder xy = new StringBuilder("{");
		int i = 0;
		for (Map.Entry<Integer, Vertex> e : g.idVertexMap.entrySet()) {
			if (i++ > 0) xy.append(", ");
			Vertex v = e.getValue();
			xy.append(q("" + e.getKey())).append(": ").append(CoreAccess.hasPoint(v) ? "[" + v.getIX() + ", " + v.getIY() + "]" : "null");
		}
		xy.append("}");
		StringBuilder adj = new StringBuilder("{");
		i = 0;
		for (Integer k : new TreeSet<>(g.adjacencySetMap.keySet())) {
			if (i++ > 0) adj.append(", ");
			adj.append(q("" + k)).append(": ").append(val(new ArrayList<>(g.adjacencySetMap.get(k))));
		}
		adj.append("}");
		return raw(obj("vertices", vs, "xy", raw(xy.toString()), "adj", raw(adj.toString()), "rows", CoreAccess.rows(g), "cols", CoreAccess.cols(g)));
	}

	static String exc(Throwable t) {
		if (t instanceof InvocationTargetException) t = t.getCause();
		return t.getClass().getName();
	}

	interface Body { Object run() throws Throwable; }

	static List<String> cases = new ArrayList<>();

	/** Run a case: seeded (seed >= 0), capture stdout, result or exception, and the next random. */
	static void add(String kind, Raw args, long seed, Body body) throws Exception {
		if (seed >= 0) seed(seed);
		PrintStream old = System.out;
		ByteArrayOutputStream bo = new ByteArrayOutputStream();
		System.setOut(new PrintStream(bo, true));
		Object res = null; String ex = null;
		try { res = body.run(); } catch (Throwable t) { ex = exc(t); }
		finally { System.out.flush(); System.setOut(old); }
		double next = Math.random();
		cases.add(obj("kind", kind, "args", args, "seed", seed, "result", res instanceof String ? raw(q((String) res)) : res,
				"exception", ex, "stdout", bo.toString(), "next", seed >= 0 ? (Object) next : null));
	}

	static void flush(String file) throws Exception {
		StringBuilder sb = new StringBuilder("[\n");
		for (int i = 0; i < cases.size(); i++) { sb.append(cases.get(i)); sb.append(i + 1 < cases.size() ? ",\n" : "\n"); }
		sb.append("]\n");
		Files.write(Paths.get(file), sb.toString().getBytes("UTF-8"));
		cases.clear();
	}

	static Graph star() { Graph g = new Graph(); g.addVertex(0, new int[]{1}); g.addVertex(1, new int[]{0, 2, 3}); g.addVertex(2, new int[]{1}); g.addVertex(3, new int[]{1}); g.finishBuildingGraph(); return g; }
	static Graph tree() {
		Graph t = new Graph(); int branches = 4, depth = 4; int[] rn = new int[branches];
		for (int i = 0; i < branches; i++) rn[i] = i * depth + 1;
		t.addVertex(0, rn);
		for (int i = 1; i <= branches; i++) for (int j = 0; j < depth; j++) { int off = (i - 1) * depth;
			if (j == depth - 1) t.addVertex(off + j + 1, new int[]{off + j}); else t.addVertex(off + j + 1, new int[]{j == 0 ? 0 : off + j, off + j + 2}); }
		t.finishBuildingGraph(); return t;
	}
	static Graph disconnected() { Graph g = new Graph(); g.addVertex(0, new int[]{1}); g.addVertex(1, new int[]{0}); g.addVertex(2, new int[]{3}); g.addVertex(3, new int[]{2}); g.finishBuildingGraph(); return g; }
	static Graph pathGraph(int... ids) { // path a-b-c... with given ids (no conversion)
		Graph g = new Graph();
		for (int i = 0; i < ids.length; i++) {
			List<Integer> nb = new ArrayList<>();
			if (i > 0) nb.add(ids[i - 1]);
			if (i + 1 < ids.length) nb.add(ids[i + 1]);
			int[] a = new int[nb.size()]; for (int k = 0; k < a.length; k++) a[k] = nb.get(k);
			g.addVertex(ids[i], a);
		}
		g.finishBuildingGraph();
		return g;
	}
	/** Converted rows x cols grid whose adjacency sets are HashSets filled self-first then ascending. */
	static Graph hashGrid(int rows, int cols) {
		Graph g = Graph.create2DGridGraph(rows, cols, true);
		for (int v = 0; v < g.vertices.length; v++) {
			Set<Integer> hs = new HashSet<>();
			hs.add(v);
			for (int u : g.adjacencySetMap.get(v)) hs.add(u);
			g.adjacencySetMap.put(v, hs);
		}
		return g;
	}
	static List<Integer> it(Set<Integer> s) { return new ArrayList<>(s); }
	static List<Integer> pl(Path p) { return p == null ? null : p.vertexList; }
	static List<List<Integer>> pls(Path[] ps) { List<List<Integer>> r = new ArrayList<>(); for (Path p : ps) r.add(pl(p)); return r; }
	static String b64(byte[] b) { return Base64.getEncoder().encodeToString(b); }

	// =================================================================== graph
	static void graphCases(String out) throws Exception {
		int[][] g4 = {{1, 1}, {1, 3}, {3, 1}, {2, 2}, {2, 3}, {3, 2}, {4, 5}, {1, 2}, {2, 1}};
		for (int[] rc : g4) for (boolean up : new boolean[]{false, true}) {
			final int r = rc[0], c = rc[1]; final boolean u = up;
			add("grid4", raw(obj("rows", r, "cols", c, "update", u)), -1, () -> dump(Graph.create2DGridGraph(r, c, u)));
			add("grid8", raw(obj("rows", r, "cols", c, "update", u)), -1, () -> dump(Graph.create2DGridGraph8Connected(r, c, u)));
		}
		for (boolean up : new boolean[]{false, true}) { final boolean u = up;
			add("grid8", raw(obj("rows", 3, "cols", 3, "update", u)), -1, () -> dump(Graph.create2DGridGraph8Connected(3, 3, u)));
			add("grid8", raw(obj("rows", 3, "cols", 4, "update", u)), -1, () -> dump(Graph.create2DGridGraph8Connected(3, 4, u))); }
		Object[][] holes = {{5, 4, 0.3}, {6, 6, 0.2}, {8, 8, 0.25}, {3, 7, 0.4}, {10, 10, 0.3}, {4, 4, 0.5}, {3, 1, 0.34}, {1, 5, 0.4},
				{3, 3, 1.0}, {3, 3, 0.99}, {2, 2, 0.5}, {3, 2, 0.3}, {2, 5, 0.3}, {7, 2, 0.4}, {15, 15, 0.2}, {3, 1, 0.67}, {1, 1, 1.0}};
		for (Object[] h : holes) for (long s : new long[]{1, 7, 42}) {
			final int r = (Integer) h[0], c = (Integer) h[1]; final double f = (Double) h[2];
			add("holes4", raw(obj("rows", r, "cols", c, "p", f)), s, () -> dump(Graph.createGeneric2DGridGraphWithHoles(r, c, f)));
			add("holes8", raw(obj("rows", r, "cols", c, "p", f)), s, () -> dump(Graph.createGeneric2DGridGraphWithHoles8Connected(r, c, f)));
		}
		add("holes4", raw(obj("rows", 32, "cols", 32, "p", 0.2)), 5, () -> dump(Graph.createGeneric2DGridGraphWithHoles(32, 32, 0.2)));
		add("holes8", raw(obj("rows", 32, "cols", 32, "p", 0.2)), 5, () -> dump(Graph.createGeneric2DGridGraphWithHoles8Connected(32, 32, 0.2)));

		// random start/goal on unconverted holes graphs
		Object[][] sgc = {{5, 4, 0.3, 4}, {6, 6, 0.2, 10}, {8, 8, 0.0, 30}, {2, 2, 0.0, 4}, {3, 3, 0.0, 9}, {24, 18, 0.25, 50}};
		for (Object[] h : sgc) for (long s : new long[]{1, 3, 99}) {
			final int r = (Integer) h[0], c = (Integer) h[1], n = (Integer) h[3]; final double f = (Double) h[2];
			add("random_sg_holes", raw(obj("rows", r, "cols", c, "p", f, "n", n)), s, () -> {
				Graph g = Graph.createGeneric2DGridGraphWithHoles(r, c, f);
				int[][] sg = Graph.getRandomStartGoal(g, n);
				return raw(obj("graph", dump(g), "sg", sg));
			});
		}
		for (long s : new long[]{5, 6, 11}) {
			add("random_sg_converted", raw(obj("rows", 3, "cols", 3, "n", 3)), s, () -> Graph.getRandomStartGoal(Graph.create2DGridGraph(3, 3, true), 3));
			add("random_sg_converted", raw(obj("rows", 4, "cols", 5, "n", 20)), s, () -> Graph.getRandomStartGoal(Graph.create2DGridGraph(4, 5, true), 20));
		}
		for (long s : new long[]{3, 11, 12}) for (int n : new int[]{3, 4, 5}) { final int nn = n;
			add("random_sg_many_grid", raw(obj("n", nn, "number", nn * nn)), s, () -> Graph.getRandomStartGoalMany(Graph.create2DGridGraph(nn, nn, true), nn * nn));
			add("random_sg_many_grid", raw(obj("n", nn, "number", nn * nn - 3)), s, () -> Graph.getRandomStartGoalMany(Graph.create2DGridGraph(nn, nn, true), nn * nn - 3));
		}
		add("random_sg_many_grid", raw(obj("n", 2, "number", 5)), 1, () -> Graph.getRandomStartGoalMany(Graph.create2DGridGraph(2, 2, true), 5));
		add("random_sg_many_grid", raw(obj("n", 2, "number", 0)), 1, () -> Graph.getRandomStartGoalMany(Graph.create2DGridGraph(2, 2, true), 0));
		for (long s : new long[]{1, 2}) add("random_sg_many_tree", raw(obj("number", 8)), s, () -> Graph.getRandomStartGoalMany(tree(), 8));

		// convertGraph with starts/goals
		for (long s : new long[]{1, 2}) add("convert", raw(obj("rows", 5, "cols", 6, "p", 0.3, "n", 6)), s, () -> {
			Graph g = Graph.createGeneric2DGridGraphWithHoles(5, 6, 0.3);
			int[][] sg = Graph.getRandomStartGoal(g, 6);
			int[][] orig = {sg[0].clone(), sg[1].clone()};
			Graph og = CoreAccess.convert(g, sg[0], sg[1]);
			return raw(obj("before", orig, "after", sg, "graph", dump(og)));
		});
		add("convert_missing_vertex", raw(obj()), -1, () -> { Graph g = pathGraph(1, 2, 3); g.adjacencySetMap.get(3).add(7); return dump(CoreAccess.convert(g, new int[0], new int[0])); });
		add("convert_bad_start", raw(obj()), -1, () -> { Graph g = pathGraph(1, 2, 3); return dump(CoreAccess.convert(g, new int[]{5}, new int[]{1})); });

		// isConnectedWithoutVertex quirks
		for (int v : new int[]{1, 2, 3}) { final int vv = v;
			add("conn_path", raw(obj("v", vv)), -1, () -> { Graph g = pathGraph(1, 2, 3); boolean b = g.isConnectedWithoutVertex(vv); return raw(obj("result", b, "graph", dump(g))); }); }
		for (int v : new int[]{0, 1, 4}) { final int vv = v;
			add("conn_grid_noself", raw(obj("v", vv)), -1, () -> { Graph g = Graph.create2DGridGraph(3, 3, true); Graph.updateAdjList(g); boolean b = g.isConnectedWithoutVertex(vv); return raw(obj("result", b, "graph", dump(g))); });
			add("conn_star", raw(obj("v", vv)), -1, () -> { Graph g = star(); boolean b = g.isConnectedWithoutVertex(vv); return raw(obj("result", b, "graph", dump(g))); }); }
		add("conn_missing", raw(obj("v", 9)), -1, () -> star().isConnectedWithoutVertex(9));
		add("conn_isolated", raw(obj("v", 2)), -1, () -> { Graph g = new Graph(); g.addVertex(2, new int[0]); g.finishBuildingGraph(); boolean b = g.isConnectedWithoutVertex(2); return raw(obj("result", b, "graph", dump(g))); });
		int[][] cw = {{0, 2, 1}, {0, 3, 1}, {2, 3, 1}, {0, 8, 4}, {1, 7, 4}, {0, 0, 4}, {3, 3, 0}};
		for (int[] t : cw) {
			add("conn_without_v3_grid3", raw(obj("a", t[0], "b", t[1], "c", t[2])), -1, () -> CoreAccess.connWithout(Graph.create2DGridGraph(3, 3, true), t[0], t[1], t[2]));
			add("conn_without_v3_star", raw(obj("a", t[0], "b", t[1], "c", t[2])), -1, () -> CoreAccess.connWithout(star(), t[0], t[1], t[2]));
		}

		// removeVertex(Vertex) (dead) and private removeVertex(x,y)
		add("remove_vertex", raw(obj("v", 0)), -1, () -> { Graph g = Graph.create2DGridGraph(2, 2, true); g.removeVertex(g.vertices[0]); return dump(g); });
		add("remove_vertex", raw(obj("v", 4)), -1, () -> { Graph g = Graph.create2DGridGraph(3, 3, true); g.removeVertex(g.vertices[4]); return dump(g); });
		add("remove_vertex_xy", raw(obj("rows", 3, "cols", 3, "x", 1, "y", 0)), -1, () -> { Graph g = Graph.create2DGridGraph(3, 3, false); CoreAccess.removeXY(g, 1, 0); return dump(g); });
		add("remove_vertex_xy_empty", raw(obj("rows", 3, "cols", 1, "x", 0, "y", 1)), -1, () -> { Graph g = Graph.create2DGridGraph8Connected(3, 1, false); CoreAccess.removeXY(g, 0, 1); return dump(g); });

		// edges (dead)
		add("edge_id", raw(obj()), -1, () -> raw(val(Arrays.asList(Graph.getEdgeId(5, 3), Graph.getEdgeId(3, 5), Graph.getEdgeId(0, 0), Graph.getEdgeId(70000, 123456),
				(long) Graph.getFirstVertexOfEdge(300000005L), (long) Graph.getSecondVertexOfEdge(300000005L), (long) Graph.getFirstVertexOfEdge(-300000005L), (long) Graph.getSecondVertexOfEdge(-300000005L),
				(long) Graph.getFirstVertexOfEdge(900000000000000007L)))));
		add("edge_set_garage", raw(obj()), -1, () -> new ArrayList<>(Problem.getLongStraightWithOneGarageProblem().graph.getEdgeSet()));
		add("edge_set_grid", raw(obj("rows", 6, "cols", 7)), -1, () -> new ArrayList<>(Graph.create2DGridGraph(6, 7, true).getEdgeSet()));
		add("edge_set_unconverted", raw(obj()), -1, () -> new ArrayList<>(Graph.create2DGridGraph(2, 2, false).getEdgeSet()));

		// update adj list (dead)
		add("update_adj_list", raw(obj()), -1, () -> { Graph g = Graph.create2DGridGraph(2, 3, true); Graph.updateAdjList(g); return dump(g); });

		// printing
		add("print_adj", raw(obj("graph", "star")), -1, () -> { star().printGraphAsAdjacenyList(); return null; });
		add("print_adj", raw(obj("graph", "tree")), -1, () -> { tree().printGraphAsAdjacenyList(); return null; });
		add("print_adj", raw(obj("graph", "grid3")), -1, () -> { Graph.create2DGridGraph(3, 3, true).printGraphAsAdjacenyList(); return null; });
		add("print_adj", raw(obj("graph", "garage")), -1, () -> { Problem.getLongStraightWithOneGarageProblem().graph.printGraphAsAdjacenyList(); return null; });
		add("print_adj", raw(obj("graph", "problem15")), 1, () -> { Problem.createGridProblem(15, 15, 0.2, 15).graph.printGraphAsAdjacenyList(); return null; });
		add("print_adj", raw(obj("graph", "unconverted")), -1, () -> { Graph.create2DGridGraph(2, 2, false).printGraphAsAdjacenyList(); return null; });
		add("print_vertex", raw(obj("graph", "grid3")), -1, () -> { Graph g = Graph.create2DGridGraph(3, 3, true); StringBuffer sb = new StringBuffer();
			for (Vertex v : g.vertices) { v.print(); System.out.print("|"); v.printVertex(); System.out.print("|"); v.printVertex(sb); sb.append("|"); } return sb.toString(); });
		add("print_vertex", raw(obj("graph", "star")), -1, () -> { Graph g = star(); StringBuffer sb = new StringBuffer();
			for (Vertex v : g.vertices) { v.print(); System.out.print("|"); v.printVertex(); System.out.print("|"); v.printVertex(sb); sb.append("|"); } return sb.toString(); });
		add("print_path", raw(obj("graph", "grid3", "path", new int[]{0, 1, 4, 5, 8})), -1, () -> { Path p = new Path(); for (int v : new int[]{0, 1, 4, 5, 8}) p.addVertex(v); p.printPath(Graph.create2DGridGraph(3, 3, true)); return null; });
		add("print_path", raw(obj("graph", "star", "path", new int[]{0, 1, 3})), -1, () -> { Path p = new Path(); for (int v : new int[]{0, 1, 3}) p.addVertex(v); p.printPath(star()); return null; });
		add("print_path", raw(obj("graph", "grid3", "path", new int[]{0, 1, 42, 5})), -1, () -> { Path p = new Path(); for (int v : new int[]{0, 1, 42, 5}) p.addVertex(v); p.printPath(Graph.create2DGridGraph(3, 3, true)); return null; });

		// test fixtures (dead)
		add("v4_test_graph", raw(obj()), -1, () -> raw(obj("graph", dump(CoreAccess.v4()), "SG", CoreAccess.SG())));
		add("bridge_test_graph", raw(obj()), -1, () -> dump(Graph.getTestGraphForBridge()));
		add("manual_star", raw(obj()), -1, () -> dump(star()));
		add("manual_tree", raw(obj()), -1, () -> dump(tree()));
		flush(out);
	}

	// ================================================================= problem
	static Raw pdump(Problem p) { return raw(obj("graph", p.graph == null ? null : dump(p.graph), "sg", p.sg)); }

	static byte[] datBytes(Problem p, String tmp) throws Exception {
		String fn = tmp + "/x.dat";
		Files.deleteIfExists(Paths.get(fn));
		Problem.writeToFile(p, fn);
		return Files.readAllBytes(Paths.get(fn));
	}
	static byte[] txtBytes(Problem p, String tmp) throws Throwable {
		String fn = tmp + "/t/x.txt";
		Files.deleteIfExists(Paths.get(fn));
		CoreAccess.writeTxt(p, fn);
		return Files.readAllBytes(Paths.get(fn));
	}
	static Raw full(Problem p, String tmp, boolean small) throws Throwable {
		byte[] d = datBytes(p, tmp), t = txtBytes(p, tmp);
		Problem r = Problem.readFromFile(tmp + "/x.dat");
		return raw(obj("problem", pdump(p), "dat_sha", sha(d), "dat_len", d.length, "dat_b64", small ? b64(d) : null,
				"txt_sha", sha(t), "txt", small ? new String(t, "UTF-8") : null, "readback", pdump(r)));
	}

	static void problemCases(String out, String tmp) throws Exception {
		Object[][] gp = {{5, 4, 0.3, 4, 42L, true}, {6, 6, 0.2, 5, 1L, true}, {8, 8, 0.0, 10, 2L, true}, {3, 7, 0.4, 5, 9L, true}, {3, 2, 0.0, 2, 1L, true},
				{15, 15, 0.2, 15, 1L, false}, {24, 18, 0.25, 50, 1L, false}, {32, 32, 0.2, 100, 3L, false}, {16, 16, 0.0, 60, 6L, false}, {24, 18, 0.3, 300, 4L, false}};
		for (Object[] a : gp) {
			final int c = (Integer) a[0], r = (Integer) a[1], n = (Integer) a[3]; final double f = (Double) a[2]; final boolean small = (Boolean) a[5];
			add("grid_problem", raw(obj("col", c, "row", r, "p", f, "n", n)), (Long) a[4], () -> full(Problem.createGridProblem(c, r, f, n), tmp, small));
		}
		Object[][] gp8 = {{6, 6, 0.2, 5, 7L, true}, {8, 8, 0.0, 10, 2L, true}, {3, 2, 0.3, 2, 1L, true}, {4, 3, 0.25, 3, 5L, true}, {32, 32, 0.2, 200, 4L, false}};
		for (Object[] a : gp8) {
			final int c = (Integer) a[0], r = (Integer) a[1], n = (Integer) a[3]; final double f = (Double) a[2]; final boolean small = (Boolean) a[5];
			add("grid_problem_8c", raw(obj("col", c, "row", r, "p", f, "n", n)), (Long) a[4], () -> full(Problem.createGridProblem8Connected(c, r, f, n), tmp, small));
		}
		for (long s : new long[]{3, 11}) for (int n : new int[]{3, 4, 5}) { final int nn = n;
			add("n2_puzzle", raw(obj("n", nn)), s, () -> full(Problem.createN2Puzzle(nn), tmp, nn == 3));
			add("n2m1_puzzle", raw(obj("n", nn)), s, () -> pdump(Problem.createN2M1Puzzle(nn)));
		}
		add("a9_puzzle", raw(obj()), -1, () -> pdump(CoreAccess.a9()));
		add("garage", raw(obj()), -1, () -> pdump(Problem.getLongStraightWithOneGarageProblem()));

		// writer crashes
		add("dat_write_null_point", raw(obj("problem", "garage")), -1, () -> {
			String fn = tmp + "/np.dat"; Files.deleteIfExists(Paths.get(fn));
			String e = null; try { Problem.writeToFile(Problem.getLongStraightWithOneGarageProblem(), fn); } catch (Throwable t) { e = exc(t); }
			byte[] b = Files.readAllBytes(Paths.get(fn)); return raw(obj("exception", e, "len", b.length, "b64", b64(b)));
		});
		add("txt_write_null_point", raw(obj("problem", "garage")), -1, () -> {
			String fn = tmp + "/npt/np.txt"; Files.deleteIfExists(Paths.get(fn));
			String e = null; try { CoreAccess.writeTxt(Problem.getLongStraightWithOneGarageProblem(), fn); } catch (Throwable t) { e = exc(t); }
			byte[] b = Files.readAllBytes(Paths.get(fn)); return raw(obj("exception", e, "len", b.length));
		});
		add("txt_write_no_parent", raw(obj()), -1, () -> { CoreAccess.writeTxt(Problem.createN2Puzzle(3), "noparent.txt"); return null; });
		for (int k : new int[]{8, 9}) { final int kk = k;
			add("dat_write_star", raw(obj("k", kk)), -1, () -> {
				int[][] pts = new int[kk + 1][]; int[][] adj = new int[kk + 1][];
				pts[0] = new int[]{0, 0}; int[] c = new int[kk + 1]; for (int i = 0; i <= kk; i++) c[i] = i; adj[0] = c;
				for (int i = 1; i <= kk; i++) { pts[i] = new int[]{i, 1}; adj[i] = new int[]{0, i}; }
				Problem p = new Problem(); p.graph = CoreAccess.withPoints(pts, adj); p.sg = new int[][]{{1, 2}, {2, 1}};
				String fn = tmp + "/star.dat"; Files.deleteIfExists(Paths.get(fn));
				String e = null; try { Problem.writeToFile(p, fn); } catch (Throwable t) { e = exc(t); }
				byte[] b = Files.readAllBytes(Paths.get(fn));
				return raw(obj("exception", e, "len", b.length, "b64", b64(b), "pts", pts, "adj", adj));
			});
		}
		add("dat_write_empty_graph", raw(obj()), -1, () -> { Problem p = new Problem(); p.graph = new Graph(); p.graph.finishBuildingGraph(); p.sg = new int[][]{{}, {}}; byte[] b = datBytes(p, tmp); return raw(obj("b64", b64(b))); });
		add("dat_write_null_sg", raw(obj()), -1, () -> { Problem p = Problem.createN2Puzzle(2); p.sg = null; byte[] b = datBytes(p, tmp); return raw(obj("b64", b64(b))); });
		add("dat_write_null_graph", raw(obj()), -1, () -> { Problem p = new Problem(); p.sg = new int[][]{{1}, {2}}; byte[] b = datBytes(p, tmp); return raw(obj("b64", b64(b))); });
		add("dat_unconverted_holes", raw(obj("rows", 4, "cols", 5, "p", 0.2)), 3, () -> {
			Problem p = new Problem(); p.graph = Graph.createGeneric2DGridGraphWithHoles(4, 5, 0.2); p.sg = Graph.getRandomStartGoal(p.graph, 3);
			byte[] b = datBytes(p, tmp); Problem r = Problem.readFromFile(tmp + "/x.dat");
			return raw(obj("problem", pdump(p), "b64", b64(b), "readback", pdump(r)));
		});
		add("dat_write_no_dir", raw(obj()), -1, () -> { Problem.writeToFile(Problem.createN2Puzzle(2), tmp + "/nodir/x.dat"); return Files.exists(Paths.get(tmp + "/nodir/x.dat")); });
		add("txt_write_unopenable", raw(obj()), -1, () -> { Files.createDirectories(Paths.get(tmp + "/isdir/sub")); CoreAccess.writeTxt(Problem.createN2Puzzle(2), tmp + "/isdir/sub"); return Files.isDirectory(Paths.get(tmp + "/isdir/sub")); });
		add("read_wrong_class", raw(obj()), -1, () -> {
			try (ObjectOutputStream oo = new ObjectOutputStream(new FileOutputStream(tmp + "/wc.dat"))) { oo.writeObject(new int[][]{{1}, {2}}); }
			return raw(obj("b64", b64(Files.readAllBytes(Paths.get(tmp + "/wc.dat"))), "problem", pdump(Problem.readFromFile(tmp + "/wc.dat"))));
		});
		// readers
		add("read_missing", raw(obj()), -1, () -> pdump(Problem.readFromFile(tmp + "/does-not-exist.dat")));
		add("read_garbage", raw(obj()), -1, () -> { Files.write(Paths.get(tmp + "/g.dat"), "0:0:0 1:1:0".getBytes()); return pdump(Problem.readFromFile(tmp + "/g.dat")); });
		add("read_truncated", raw(obj()), -1, () -> { byte[] b = datBytes(Problem.createN2Puzzle(3), tmp); Files.write(Paths.get(tmp + "/tr.dat"), Arrays.copyOf(b, b.length - 20)); return pdump(Problem.readFromFile(tmp + "/tr.dat")); });
		add("read_truncated_graph", raw(obj()), -1, () -> { byte[] b = datBytes(Problem.createN2Puzzle(3), tmp); Files.write(Paths.get(tmp + "/tr2.dat"), Arrays.copyOf(b, 400)); return pdump(Problem.readFromFile(tmp + "/tr2.dat")); });
		flush(out);
	}

	static void familyCase(String out, String tmp, String fam, long s) throws Exception {
		String dir = tmp + "/fam-" + fam + "/";
		deleteTree(Paths.get(dir));
		Files.createDirectories(Paths.get(dir));
		add("family", raw(obj("family", fam)), s, () -> {
			CoreAccess.setFolder(dir);
			CoreAccess.family(fam);
			TreeMap<String, byte[]> files = new TreeMap<>();
			try (DirectoryStream<java.nio.file.Path> ds = Files.newDirectoryStream(Paths.get(dir))) {
				for (java.nio.file.Path p : ds) files.put(p.getFileName().toString(), Files.readAllBytes(p));
			}
			MessageDigest md = MessageDigest.getInstance("SHA-256");
			for (Map.Entry<String, byte[]> e : files.entrySet()) {
				md.update(e.getKey().getBytes("UTF-8")); md.update((byte) 0);
				md.update(sha(e.getValue()).getBytes("UTF-8")); md.update((byte) 0);
			}
			List<String> names = new ArrayList<>(files.keySet());
			return raw(obj("count", files.size(), "sha", hex(md.digest()), "first", names.subList(0, Math.min(6, names.size()))));
		});
		deleteTree(Paths.get(dir));
	}
	static void deleteTree(java.nio.file.Path p) throws IOException {
		if (!Files.exists(p)) return;
		Files.walk(p).sorted(Comparator.reverseOrder()).forEach(x -> x.toFile().delete());
	}

	// ============================================================== pathfinder
	static void pathfinderCases(String out) throws Exception {
		Graph star = star(), grid3 = Graph.create2DGridGraph(3, 3, true), dis = disconnected();
		add("dist", raw(obj("graph", "star", "start", 0)), -1, () -> PathFinder.getDistancesToAllVertices(star, 0, null));
		add("dist", raw(obj("graph", "star", "start", 2)), -1, () -> PathFinder.getDistancesToAllVertices(star, 2, null));
		for (int s : new int[]{0, 4, 8}) { final int ss = s; add("dist", raw(obj("graph", "grid3", "start", ss)), -1, () -> PathFinder.getDistancesToAllVertices(grid3, ss, null)); }
		add("dist", raw(obj("graph", "disconnected", "start", 0)), -1, () -> PathFinder.getDistancesToAllVertices(dis, 0, null));
		add("dist_prefilled", raw(obj("graph", "disconnected", "start", 0, "fill", 99)), -1, () -> { int[] a = {99, 99, 99, 99}; int[] r = PathFinder.getDistancesToAllVertices(dis, 0, a); return raw(obj("result", r, "same", r == a)); });
		add("dist", raw(obj("graph", "tree", "start", 7)), -1, () -> PathFinder.getDistancesToAllVertices(tree(), 7, null));
		add("dist", raw(obj("graph", "hash6x6", "start", 14)), -1, () -> PathFinder.getDistancesToAllVertices(hashGrid(6, 6), 14, null));
		add("dist", raw(obj("graph", "unconverted2x2", "start", 1)), -1, () -> PathFinder.getDistancesToAllVertices(Graph.create2DGridGraph(2, 2, false), 1, null));
		add("dist", raw(obj("graph", "star", "start", 4)), -1, () -> PathFinder.getDistancesToAllVertices(star, 4, null));
		add("dist_problem", raw(obj("col", 12, "row", 10, "p", 0.25, "n", 3)), 8, () -> {
			Problem p = Problem.createGridProblem(12, 10, 0.25, 3); List<int[]> r = new ArrayList<>();
			for (int a = 0; a < 3; a++) { r.add(PathFinder.getDistancesToAllVertices(p.graph, p.sg[0][a], null)); r.add(PathFinder.getDistancesToAllVertices(p.graph, p.sg[1][a], null)); }
			return raw(val(r));
		});

		int[][] ranges = {{0, 0}, {0, 1}, {1, 1}, {2, 2}, {1, 2}, {3, 4}, {0, 100}, {5, 3}, {7, 9}, {-1, 2}, {4, 4}};
		for (int[] rg : ranges) {
			add("range", raw(obj("graph", "grid5", "start", 12, "dmin", rg[0], "dmax", rg[1])), -1, () -> it(PathFinder.findGoalsInDistanceRange(Graph.create2DGridGraph(5, 5, true), 12, rg[0], rg[1])));
			add("range", raw(obj("graph", "grid7", "start", 24, "dmin", rg[0], "dmax", rg[1])), -1, () -> it(PathFinder.findGoalsInDistanceRange(Graph.create2DGridGraph(7, 7, true), 24, rg[0], rg[1])));
			add("range", raw(obj("graph", "hash6x6", "start", 14, "dmin", rg[0], "dmax", rg[1])), -1, () -> it(PathFinder.findGoalsInDistanceRange(hashGrid(6, 6), 14, rg[0], rg[1])));
		}
		int[][] bigRanges = {{3, 3}, {5, 7}, {10, 12}, {1, 40}, {20, 20}};
		for (int[] rg : bigRanges) for (int st : new int[]{0, 333, 700}) {
			add("range_problem32", raw(obj("start", st, "dmin", rg[0], "dmax", rg[1])), 3, () -> {
				Problem p = Problem.createGridProblem(32, 32, 0.2, 1); return it(PathFinder.findGoalsInDistanceRange(p.graph, st, rg[0], rg[1]));
			});
		}
		add("range", raw(obj("graph", "star", "start", 0, "dmin", 2, "dmax", 2)), -1, () -> it(PathFinder.findGoalsInDistanceRange(star, 0, 2, 2)));
		add("range", raw(obj("graph", "tree", "start", 0, "dmin", 2, "dmax", 3)), -1, () -> it(PathFinder.findGoalsInDistanceRange(tree(), 0, 2, 3)));
		add("range", raw(obj("graph", "disconnected", "start", 0, "dmin", 0, "dmax", 5)), -1, () -> it(PathFinder.findGoalsInDistanceRange(dis, 0, 0, 5)));

		// randomized BFS
		Object[][] sp = {{"grid3", 0, 8}, {"grid6x7", 0, 41}, {"grid6x7", 16, 28}, {"grid5", 0, 24}, {"star", 0, 3}, {"star", 2, 3}, {"tree", 4, 16}, {"hash6x6", 0, 35}, {"hash6x6", 14, 21},
				{"grid6x7", 5, 5}, {"disconnected", 0, 3}, {"grid8", 0, 63}};
		for (Object[] c : sp) for (long s = 0; s < 6; s++) for (boolean h : new boolean[]{false, true}) {
			final String gname = (String) c[0]; final int a = (Integer) c[1], b = (Integer) c[2]; final boolean hh = h;
			add("one_path", raw(obj("graph", gname, "start", a, "goal", b, "heuristic", hh)), s, () -> {
				PathFinder.bHeuristic = hh;
				try { return pl(PathFinder.findOneShortestPath(graphByName(gname), a, b, null)); } finally { PathFinder.bHeuristic = false; }
			});
		}
		for (long s : new long[]{1, 2, 3, 4}) for (boolean h : new boolean[]{false, true}) { final boolean hh = h;
			add("paths_problem", raw(obj("col", 12, "row", 10, "p", 0.25, "n", 8, "heuristic", hh)), s, () -> {
				Problem p = Problem.createGridProblem(12, 10, 0.25, 8);
				PathFinder.bHeuristic = hh;
				try { return raw(obj("sg", p.sg, "paths", pls(PathFinder.findShortestPaths(p.graph, p.sg[0], p.sg[1])))); } finally { PathFinder.bHeuristic = false; }
			});
			add("lb_problem", raw(obj("col", 8, "row", 9, "p", 0.2, "n", 6, "heuristic", hh)), s, () -> {
				Problem p = Problem.createGridProblem(8, 9, 0.2, 6);
				PathFinder.bHeuristic = hh;
				try {
					int m = PathFinder.getMakespanLowerBound(p.graph, p.sg[0], p.sg[1]);
					int t = PathFinder.getTotalTimeLowerBound(p.graph, p.sg[0], p.sg[1]);
					return raw(obj("makespan_lb", m, "total_time_lb", t));
				} finally { PathFinder.bHeuristic = false; }
			});
		}
		add("lb", raw(obj("graph", "star", "starts", new int[]{0, 3}, "goals", new int[]{3, 0})), 42, () -> new int[]{PathFinder.getMakespanLowerBound(star, new int[]{0, 3}, new int[]{3, 0}), PathFinder.getTotalTimeLowerBound(star, new int[]{0, 3}, new int[]{3, 0})});
		add("lb", raw(obj("graph", "grid3", "starts", new int[]{0, 4}, "goals", new int[]{8, 4})), 1, () -> new int[]{PathFinder.getMakespanLowerBound(grid3, new int[]{0, 4}, new int[]{8, 4}), PathFinder.getTotalTimeLowerBound(grid3, new int[]{0, 4}, new int[]{8, 4})});
		add("lb", raw(obj("graph", "grid3", "starts", new int[]{}, "goals", new int[]{})), 1, () -> new int[]{PathFinder.getMakespanLowerBound(grid3, new int[]{}, new int[]{}), PathFinder.getTotalTimeLowerBound(grid3, new int[]{}, new int[]{})});
		add("lb_makespan", raw(obj("graph", "disconnected", "starts", new int[]{0, 0}, "goals", new int[]{1, 3})), 5, () -> PathFinder.getMakespanLowerBound(dis, new int[]{0, 0}, new int[]{1, 3}));
		add("lb_total_time", raw(obj("graph", "disconnected", "starts", new int[]{0, 0}, "goals", new int[]{1, 3})), 5, () -> PathFinder.getTotalTimeLowerBound(dis, new int[]{0, 0}, new int[]{1, 3}));
		add("total_distance", raw(obj("paths", new int[][]{{0, 1, 1, 2}, {3, 3, 3, 3}, {1, 0, 1, 0}})), -1, () -> PathFinder.getTotalDistance(new int[][]{{0, 1, 1, 2}, {3, 3, 3, 3}, {1, 0, 1, 0}}));
		add("total_distance", raw(obj("paths", new int[][]{})), -1, () -> PathFinder.getTotalDistance(new int[][]{}));
		add("total_path_length", raw(obj("graph", "star", "starts", new int[]{0, 3}, "goals", new int[]{3, 0})), 42, () -> PathFinder.getTotalDistance(PathFinder.findShortestPaths(star, new int[]{0, 3}, new int[]{3, 0})));
		flush(out);
	}

	static Graph graphByName(String n) {
		switch (n) {
		case "grid3": return Graph.create2DGridGraph(3, 3, true);
		case "grid5": return Graph.create2DGridGraph(5, 5, true);
		case "grid8": return Graph.create2DGridGraph(8, 8, true);
		case "grid6x7": return Graph.create2DGridGraph(6, 7, true);
		case "star": return star();
		case "tree": return tree();
		case "hash6x6": return hashGrid(6, 6);
		case "disconnected": return disconnected();
		}
		throw new IllegalArgumentException(n);
	}

	public static void main(String[] a) throws Exception {
		String out = a[0], tmp = a[1];
		Files.createDirectories(Paths.get(out));
		Files.createDirectories(Paths.get(tmp));
		String what = a.length > 2 ? a[2] : "graph,problem,pathfinder";
		if (what.contains("graph")) graphCases(out + "/graph.json");
		if (what.contains("problem")) problemCases(out + "/problem.json", tmp);
		if (what.contains("pathfinder")) pathfinderCases(out + "/pathfinder.json");
		if (what.contains("families")) {
			String[] fams = {"n2puzzle", "crowded", "32x32", "32x32-8c", "24x18", "crowded-8c"};
			for (int i = 0; i < fams.length; i++) familyCase(out, tmp, fams[i], 101 + i);
			flush(out + "/families.json");
		}
	}
}
