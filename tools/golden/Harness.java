package projects.multipath.ILP;

// Golden-fixture harness for our original Java code (optimal-mrppg-journal, T-RO 2016).
//
// It is compiled into the package projects.multipath.ILP next to the (unmodified) original
// sources, so that package-private and protected members are directly reachable; private
// members are reached by reflection.  See tools/golden/README.md for the record format.
//
// Usage: java --add-opens java.base/java.lang=ALL-UNNAMED projects.multipath.ILP.Harness \
//            <cases.jsonl> <out.jsonl> <progress.txt> <workdir>
//
// Each input line is a case {"id","fn","args","seed","statics","budget"}.  For each case the
// harness resets all static flags of the original classes to their declared defaults, applies
// "statics", seeds the global Math.random() generator with "seed", runs the call in a worker
// thread and appends one JSON line to <out.jsonl>.  If the call does not return within
// "budget" seconds, a {"hang": true} record is written and the JVM exits with code 3; the
// driver (record.py) restarts the harness with the remaining cases.
//
// The gurobi imports below are rewritten by build.sh when the jar uses com.gurobi.gurobi.

import gurobi.GRB;
import gurobi.GRBConstr;
import gurobi.GRBEnv;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBVar;

import java.awt.geom.Point2D;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintStream;
import java.io.Writer;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Base64;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.TreeSet;

import projects.multipath.advanced.Graph;
import projects.multipath.advanced.Path;
import projects.multipath.advanced.Problem;
import projects.multipath.advanced.Vertex;

public class Harness {

    static final int EXIT_HANG = 3;
    static final int MAX_CAPTURE = 20000;      // chars of stdout/stderr kept per case
    static final int MAX_DUMP = 20000;         // model dumps longer than this are stored as hash only
    static final int DRAW_SEARCH_LIMIT = 20000000;
    static final String DEFAULT_FILE_FOLDER = "D:\\temp\\data\\dmrpp\\";
    static final String DEFAULT_BASE_PATH = "D:/temp/data/dmrpp/";

    static File workDir;
    static String caseDirPath = null;
    static ByteArrayOutputStream capOut, capErr;
    static PrintStream realErr;

    // ------------------------------------------------------------------------------------
    // Driver loop
    // ------------------------------------------------------------------------------------

    public static void main(String[] argv) throws Exception {
        if (argv.length < 4) {
            System.err.println("usage: Harness <cases.jsonl> <out.jsonl> <progress.txt> <workdir>");
            System.exit(2);
        }
        realErr = System.err;
        workDir = new File(argv[3]).getAbsoluteFile();
        workDir.mkdirs();
        List<String> lines = Files.readAllLines(new File(argv[0]).toPath(), StandardCharsets.UTF_8);
        File progress = new File(argv[2]);
        Writer w = new OutputStreamWriter(new FileOutputStream(argv[1], true), StandardCharsets.UTF_8);
        for (String line : lines) {
            if (line.trim().isEmpty()) continue;
            @SuppressWarnings("unchecked")
            Map<String, Object> c = (Map<String, Object>) Json.parse(line);
            Files.write(progress.toPath(), String.valueOf(c.get("id")).getBytes(StandardCharsets.UTF_8));
            Map<String, Object> rec = runCase(c, w);
            w.write(Json.emit(rec));
            w.write("\n");
            w.flush();
        }
        w.close();
        // Lingering non-daemon threads of the original (e.g. PuzzleSolver workers) must not keep
        // the JVM alive.
        Runtime.getRuntime().halt(0);
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> runCase(Map<String, Object> c, Writer w) throws Exception {
        final String id = String.valueOf(c.get("id"));
        final String fn = (String) c.get("fn");
        final Map<String, Object> args = c.get("args") == null ? new LinkedHashMap<String, Object>()
                : (Map<String, Object>) c.get("args");
        long seed = c.get("seed") == null ? 0L : ((Number) c.get("seed")).longValue();
        double budget = c.get("budget") == null ? 120.0 : ((Number) c.get("budget")).doubleValue();

        File caseDir = new File(workDir, "case-" + id.replaceAll("[^A-Za-z0-9_.-]", "_"));
        deleteRecursively(caseDir);
        caseDir.mkdirs();
        caseDirPath = caseDir.getAbsolutePath();

        resetStatics(caseDir);
        applyStatics((Map<String, Object>) c.get("statics"));
        final Map<String, Object> flagsBefore = readFlags();

        capOut = new ByteArrayOutputStream();
        capErr = new ByteArrayOutputStream();
        System.setOut(new PrintStream(capOut, true, "UTF-8"));
        System.setErr(new PrintStream(capErr, true, "UTF-8"));

        final Ctx ctx = new Ctx(caseDir);
        final Object[] res = new Object[1];
        final Throwable[] err = new Throwable[1];
        Set<Thread> threadsBefore = new HashSet<Thread>(Thread.getAllStackTraces().keySet());

        seedMath(seed);
        Thread t = new Thread(null, new Runnable() {
            public void run() {
                try {
                    res[0] = dispatch(fn, args, ctx);
                } catch (Throwable e) {
                    err[0] = unwrap(e);
                }
            }
        }, "harness-case", 512L << 20);
        t.setDaemon(true);
        long t0 = System.nanoTime();
        t.start();
        t.join((long) (budget * 1000));

        Map<String, Object> rec = new LinkedHashMap<String, Object>();
        rec.put("id", id);
        if (t.isAlive()) {
            rec.put("hang", Boolean.TRUE);
            StackTraceElement[] st = t.getStackTrace();
            List<Object> hs = frames(st, 2, false);         // where it is spinning (may be JDK code)
            for (Object f : frames(st, 10, true)) if (!hs.contains(f)) hs.add(f);  // + original frames
            rec.put("hang_stack", hs);
            rec.put("stdout", captured(capOut));
            rec.put("stderr", captured(capErr));
            rec.put("elapsed_ms", (System.nanoTime() - t0) / 1000000L);
            w.write(Json.emit(rec));
            w.write("\n");
            w.flush();
            w.close();
            Runtime.getRuntime().halt(EXIT_HANG);
        }
        long elapsed = (System.nanoTime() - t0) / 1000000L;
        // Wait (bounded) for threads started by the call, so that they cannot touch Math.random
        // or stdout after this point.
        long waitUntil = System.currentTimeMillis() + 10000;
        for (Thread th : Thread.getAllStackTraces().keySet()) {
            if (threadsBefore.contains(th) || th == t || th.isDaemon()) continue;
            long left = waitUntil - System.currentTimeMillis();
            if (left > 0) th.join(left);
        }
        double next = Math.random();

        if (err[0] != null) {
            rec.put("exception", exceptionJson(err[0]));
        } else {
            rec.put("result", toJson(res[0]));
        }
        if (!ctx.mutated.isEmpty()) rec.put("mutated", ctx.mutated);
        if (!ctx.extra.isEmpty()) rec.put("extra", ctx.extra);
        rec.put("stdout", captured(capOut));
        rec.put("stderr", captured(capErr));
        rec.put("rng_next", next);
        rec.put("draws", countDraws(seed, next));
        Map<String, Object> flagsAfter = readFlags();
        Map<String, Object> changed = new LinkedHashMap<String, Object>();
        for (Map.Entry<String, Object> e : flagsAfter.entrySet()) {
            if (!e.getValue().equals(flagsBefore.get(e.getKey()))) changed.put(e.getKey(), e.getValue());
        }
        if (!changed.isEmpty()) rec.put("statics_after", changed);
        Object files = collectFiles(caseDir, ctx.skipFiles);
        if (files != null) rec.put("files", files);
        rec.put("elapsed_ms", elapsed);
        System.setOut(new PrintStream(new ByteArrayOutputStream()));
        System.setErr(realErr);
        deleteRecursively(caseDir);
        return rec;
    }

    static Throwable unwrap(Throwable e) {
        while ((e instanceof InvocationTargetException) && e.getCause() != null) e = e.getCause();
        return e;
    }

    static Map<String, Object> exceptionJson(Throwable e) {
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        m.put("class", e.getClass().getName());
        m.put("message", scrub(e.getMessage()));
        m.put("at", frames(e.getStackTrace(), 8, true));
        return m;
    }

    /** Frames of the original code (projects.multipath.*, excluding this harness). */
    static List<Object> frames(StackTraceElement[] st, int max, boolean originalOnly) {
        List<Object> r = new ArrayList<Object>();
        for (StackTraceElement f : st) {
            String cls = f.getClassName();
            boolean orig = cls.startsWith("projects.multipath.") && !cls.startsWith("projects.multipath.ILP.Harness");
            if (originalOnly && !orig) continue;
            if (!originalOnly && !orig && !cls.startsWith("java.")) continue;
            r.add(cls.substring(cls.lastIndexOf('.') + 1) + "." + f.getMethodName() + ":" + f.getLineNumber());
            if (r.size() >= max) break;
        }
        return r;
    }

    static String scrub(String s) {
        if (s == null) return null;
        if (caseDirPath != null) s = s.replace(caseDirPath, "$CASEDIR");
        if (workDir != null) s = s.replace(workDir.getAbsolutePath(), "$WORKDIR");
        return s;
    }

    static String captured(ByteArrayOutputStream b) {
        String s = new String(b.toByteArray(), StandardCharsets.UTF_8);
        s = scrub(s);
        if (s.length() > MAX_CAPTURE) s = s.substring(0, MAX_CAPTURE) + "\n<truncated " + s.length() + " chars>";
        return s;
    }

    static void seedMath(long seed) throws Exception {
        Math.random(); // make sure the holder class is initialised
        Class<?> h = Class.forName("java.lang.Math$RandomNumberGeneratorHolder");
        Field f = h.getDeclaredField("randomNumberGenerator");
        f.setAccessible(true);
        ((Random) f.get(null)).setSeed(seed);
    }

    /** Number of Math.random() draws the call consumed: position of 'next' in the seeded stream. */
    static long countDraws(long seed, double next) {
        Random r = new Random(seed);
        for (int k = 0; k < DRAW_SEARCH_LIMIT; k++) {
            if (r.nextDouble() == next) return k;
        }
        return -1;
    }

    // ------------------------------------------------------------------------------------
    // Static state of the original classes
    // ------------------------------------------------------------------------------------

    static final String[][] FLAGS = {
        {"MultiagentGraphSolverGurobiTime", "bPrintPaths", "true"},
        {"MultiagentGraphSolverGurobiTime", "bDebugInfo", "true"},
        {"MultiagentGraphSolverGurobiTimeR", "bPrintPaths", "true"},
        {"MultiagentGraphSolverGurobiTimeR", "bDebugInfo", "true"},
        {"MultiagentGraphSolverGurobiTotalTime", "bPrintPaths", "true"},
        {"MultiagentGraphSolverGurobiTotalTime", "bDebugInfo", "true"},
        {"MultiagentGraphSolverGurobiTotalTime", "bReturnAnySolution", "false"},
        {"MultiagentGraphSolverGurobiDistance", "bPrintPaths", "false"},
        {"MultiagentGraphSolverGurobiDistance", "bDebugInfo", "true"},
        {"MultiagentGraphSolverGurobiDistance", "bReturnAnySolution", "false"},
        {"PathFinder", "bHeuristic", "false"},
    };

    static Class<?> cls(String simple) throws ClassNotFoundException {
        try {
            return Class.forName("projects.multipath.ILP." + simple);
        } catch (ClassNotFoundException e) {
            return Class.forName("projects.multipath.advanced." + simple);
        }
    }

    static Field staticField(String qualified) throws Exception {
        int dot = qualified.lastIndexOf('.');
        Field f = cls(qualified.substring(0, dot)).getDeclaredField(qualified.substring(dot + 1));
        f.setAccessible(true);
        if (!Modifier.isStatic(f.getModifiers())) throw new IllegalArgumentException("not static: " + qualified);
        return f;
    }

    static void resetStatics(File caseDir) throws Exception {
        for (String[] fl : FLAGS) staticField(fl[0] + "." + fl[1]).setBoolean(null, Boolean.parseBoolean(fl[2]));
        staticField("Problem.FILE_FOLDER").set(null, DEFAULT_FILE_FOLDER);
        Main.basePath = caseDir.getAbsolutePath() + "/";
        Main.currentTest = null;
        Main.subTotal = 0;
        Main.pathLengths = 0;
        Main.solvedInstances = 0;
        Main.totalTime = 0;
        Main.successCount = 0;
        Main.goalsReached = 0;
        Main.pathLength = 0;
        Main.expPathLength = 0;
        Main.finishingTimeSet = new TreeSet<Double>();
        Main.makeSpanTotal = 0;
    }

    static void applyStatics(Map<String, Object> st) throws Exception {
        if (st == null) return;
        for (Map.Entry<String, Object> e : st.entrySet()) {
            Field f = staticField(e.getKey());
            Object v = e.getValue();
            Class<?> t = f.getType();
            if (t == boolean.class) f.setBoolean(null, (Boolean) v);
            else if (t == int.class) f.setInt(null, ((Number) v).intValue());
            else if (t == long.class) f.setLong(null, ((Number) v).longValue());
            else if (t == double.class) f.setDouble(null, toDouble(v));
            else if (t == String.class) f.set(null, v == null ? null : subst((String) v));
            else throw new IllegalArgumentException("unsupported static type " + t + " for " + e.getKey());
        }
    }

    static Map<String, Object> readFlags() throws Exception {
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        for (String[] fl : FLAGS) m.put(fl[0] + "." + fl[1], staticField(fl[0] + "." + fl[1]).getBoolean(null));
        return m;
    }

    static String subst(String s) {
        if (s == null || caseDirPath == null) return s;
        return s.replace("$CASEDIR", caseDirPath);
    }

    // ------------------------------------------------------------------------------------
    // Files written by a call
    // ------------------------------------------------------------------------------------

    static void deleteRecursively(File f) {
        if (!f.exists()) return;
        File[] kids = f.listFiles();
        if (kids != null) for (File k : kids) deleteRecursively(k);
        f.delete();
    }

    static void listFiles(File root, File f, List<File> out) {
        File[] kids = f.listFiles();
        if (kids == null) return;
        Arrays.sort(kids);
        for (File k : kids) {
            if (k.isDirectory()) listFiles(root, k, out);
            else out.add(k);
        }
    }

    static String rel(File root, File f) {
        return root.toPath().relativize(f.toPath()).toString();
    }

    static Object collectFiles(File root, Set<String> skip) throws Exception {
        List<File> fs = new ArrayList<File>();
        listFiles(root, root, fs);
        List<File> keep = new ArrayList<File>();
        for (File f : fs) if (!skip.contains(rel(root, f))) keep.add(f);
        if (keep.isEmpty()) return null;
        if (keep.size() > 20) {
            Map<String, Object> m = new LinkedHashMap<String, Object>();
            m.put("count", keep.size());
            m.put("sha256", digestFiles(root, keep));
            return m;
        }
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        for (File f : keep) m.put(rel(root, f), fileJson(f, MAX_CAPTURE, 4096));
        return m;
    }

    static Map<String, Object> fileJson(File f, int maxText, int maxB64) throws Exception {
        byte[] b = Files.readAllBytes(f.toPath());
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        m.put("size", b.length);
        m.put("sha256", sha256(b));
        String text = utf8OrNull(b);
        if (text != null && b.length <= maxText) m.put("text", scrub(text));
        else if (b.length <= maxB64) m.put("base64", Base64.getEncoder().encodeToString(b));
        return m;
    }

    static String utf8OrNull(byte[] b) {
        for (byte x : b) if (x == 0) return null;
        try {
            return StandardCharsets.UTF_8.newDecoder().decode(java.nio.ByteBuffer.wrap(b)).toString();
        } catch (Exception e) {
            return null;
        }
    }

    /** SHA-256 over "name\0size\0bytes" of each file in name order. */
    static String digestFiles(File root, List<File> files) throws Exception {
        MessageDigest md = MessageDigest.getInstance("SHA-256");
        List<String> names = new ArrayList<String>();
        Map<String, File> byName = new HashMap<String, File>();
        for (File f : files) {
            names.add(rel(root, f));
            byName.put(rel(root, f), f);
        }
        java.util.Collections.sort(names);
        for (String n : names) {
            byte[] b = Files.readAllBytes(byName.get(n).toPath());
            md.update(n.getBytes(StandardCharsets.UTF_8));
            md.update((byte) 0);
            md.update(Integer.toString(b.length).getBytes(StandardCharsets.UTF_8));
            md.update((byte) 0);
            md.update(b);
        }
        return hex(md.digest());
    }

    static String sha256(byte[] b) throws Exception {
        return hex(MessageDigest.getInstance("SHA-256").digest(b));
    }

    static String hex(byte[] d) {
        StringBuilder sb = new StringBuilder();
        for (byte x : d) sb.append(String.format("%02x", x));
        return sb.toString();
    }

    /** Digest of a batch-generation folder: per kind (.txt/.dat) counts and hashes. */
    static Map<String, Object> batchDigest(File folder, int samples) throws Exception {
        List<File> fs = new ArrayList<File>();
        listFiles(folder, folder, fs);
        List<File> txt = new ArrayList<File>(), dat = new ArrayList<File>();
        for (File f : fs) (f.getName().endsWith(".dat") ? dat : txt).add(f);
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        m.put("nfiles", fs.size());
        m.put("ntxt", txt.size());
        m.put("ndat", dat.size());
        m.put("sha256_txt", digestFiles(folder, txt));
        m.put("sha256_dat", digestFiles(folder, dat));
        List<String> names = new ArrayList<String>();
        for (File f : fs) names.add(rel(folder, f));
        java.util.Collections.sort(names);
        MessageDigest md = MessageDigest.getInstance("SHA-256");
        for (String n : names) {
            md.update(n.getBytes(StandardCharsets.UTF_8));
            md.update((byte) '\n');
        }
        m.put("sha256_names", hex(md.digest()));
        m.put("first_names", names.subList(0, Math.min(6, names.size())));
        Map<String, Object> s = new LinkedHashMap<String, Object>();
        int k = 0;
        for (String n : names) {
            if (!n.endsWith(".txt")) continue;
            s.put(n, new String(Files.readAllBytes(new File(folder, n).toPath()), StandardCharsets.UTF_8));
            if (++k >= samples) break;
        }
        m.put("samples", s);
        return m;
    }

    // ------------------------------------------------------------------------------------
    // Argument conversion (JSON -> Java) with mutation tracking
    // ------------------------------------------------------------------------------------

    static class Ctx {
        final File dir;
        Map<String, Object> conv = new LinkedHashMap<String, Object>();
        Map<String, String> snap = new LinkedHashMap<String, String>();
        Map<String, Object> mutated = new LinkedHashMap<String, Object>();
        Map<String, Object> extra = new LinkedHashMap<String, Object>();
        Set<String> skipFiles = new HashSet<String>();
        PathPlanner pp = null;
        HeuristicPathPlanner hpp = null;
        boolean shared = false;

        Ctx(File dir) {
            this.dir = dir;
        }

        PathPlanner pp() {
            if (pp == null || !shared) pp = new PathPlanner();
            return pp;
        }

        HeuristicPathPlanner hpp() {
            if (hpp == null || !shared) hpp = new HeuristicPathPlanner();
            return hpp;
        }

        void track(String k, Object o) {
            conv.put(k, o);
            snap.put(k, Json.emit(toJson(o)));
        }

        Map<String, Object> diff() {
            Map<String, Object> m = new LinkedHashMap<String, Object>();
            for (Map.Entry<String, Object> e : conv.entrySet()) {
                Object j = toJson(e.getValue());
                if (!Json.emit(j).equals(snap.get(e.getKey()))) m.put(e.getKey(), j);
            }
            conv = new LinkedHashMap<String, Object>();
            snap = new LinkedHashMap<String, String>();
            return m;
        }
    }

    static Object req(Map<String, Object> a, String k) {
        if (!a.containsKey(k)) throw new IllegalArgumentException("missing argument '" + k + "'");
        return a.get(k);
    }

    static int I(Map<String, Object> a, String k) {
        return ((Number) req(a, k)).intValue();
    }

    static long L(Map<String, Object> a, String k) {
        return ((Number) req(a, k)).longValue();
    }

    static boolean B(Map<String, Object> a, String k) {
        return (Boolean) req(a, k);
    }

    static double D(Map<String, Object> a, String k) {
        return toDouble(req(a, k));
    }

    static String S(Map<String, Object> a, String k) {
        Object o = req(a, k);
        return o == null ? null : subst((String) o);
    }

    static double toDouble(Object o) {
        if (o instanceof Number) return ((Number) o).doubleValue();
        String s = (String) o;
        if (s.equals("NaN")) return Double.NaN;
        if (s.equals("Infinity")) return Double.POSITIVE_INFINITY;
        if (s.equals("-Infinity")) return Double.NEGATIVE_INFINITY;
        return Double.parseDouble(s);
    }

    static int[] toIntArr(Object o) {
        if (o == null) return null;
        List<?> l = (List<?>) o;
        int[] r = new int[l.size()];
        for (int i = 0; i < r.length; i++) r[i] = ((Number) l.get(i)).intValue();
        return r;
    }

    static int[][] toIntArr2(Object o) {
        if (o == null) return null;
        List<?> l = (List<?>) o;
        int[][] r = new int[l.size()][];
        for (int i = 0; i < r.length; i++) r[i] = toIntArr(l.get(i));
        return r;
    }

    static int[] IA(Ctx c, Map<String, Object> a, String k) {
        int[] r = toIntArr(req(a, k));
        if (r != null) c.track(k, r);
        return r;
    }

    static int[][] IA2(Ctx c, Map<String, Object> a, String k) {
        int[][] r = toIntArr2(req(a, k));
        if (r != null) c.track(k, r);
        return r;
    }

    static Integer[] IGA(Map<String, Object> a, String k) {
        int[] r = toIntArr(req(a, k));
        if (r == null) return null;
        Integer[] x = new Integer[r.length];
        for (int i = 0; i < r.length; i++) x[i] = r[i];
        return x;
    }

    static Set<Integer> SET(Ctx c, Map<String, Object> a, String k) {
        Object o = req(a, k);
        if (o == null) return null;
        Set<Integer> s = new HashSet<Integer>();
        for (Object x : (List<?>) o) s.add(((Number) x).intValue());
        c.track(k, s);
        return s;
    }

    static Path[] toPathArr(Object o) {
        if (o == null) return null;
        List<?> l = (List<?>) o;
        Path[] r = new Path[l.size()];
        for (int i = 0; i < r.length; i++) {
            if (l.get(i) == null) continue;
            r[i] = new Path();
            for (Object x : (List<?>) l.get(i)) r[i].addVertex(((Number) x).intValue());
        }
        return r;
    }

    static Path[] PA(Ctx c, Map<String, Object> a, String k) {
        Path[] r = toPathArr(req(a, k));
        if (r != null) c.track(k, r);
        return r;
    }

    @SuppressWarnings("unchecked")
    static Graph toGraph(Object o) throws Exception {
        if (o == null) return null;
        Map<String, Object> m = (Map<String, Object>) o;
        Graph g = new Graph();
        List<Object> vs = (List<Object>) m.get("vertices");
        if (vs != null) {
            Vertex[] arr = new Vertex[vs.size()];
            for (int i = 0; i < arr.length; i++) {
                List<Object> v = (List<Object>) vs.get(i);
                int id = ((Number) v.get(0)).intValue();
                Vertex vx;
                if (v.size() >= 3 && v.get(1) != null)
                    vx = new Vertex(new Point2D.Double(toDouble(v.get(1)), toDouble(v.get(2))), id);
                else
                    vx = new Vertex(id);
                arr[i] = vx;
                g.verticeSet.add(vx);
                g.idVertexMap.put(id, vx);
            }
            g.vertices = arr;
        }
        // Vertices present in idVertexMap/verticeSet but not in the 'vertices' array (a stale
        // array, as after addVertex without finishBuildingGraph).
        List<Object> extra = (List<Object>) m.get("idVertexMap_extra");
        if (extra != null) {
            for (Object o2 : extra) {
                List<Object> v = (List<Object>) o2;
                int id = ((Number) v.get(0)).intValue();
                Vertex vx = (v.size() >= 3 && v.get(1) != null)
                        ? new Vertex(new Point2D.Double(toDouble(v.get(1)), toDouble(v.get(2))), id) : new Vertex(id);
                g.verticeSet.add(vx);
                g.idVertexMap.put(id, vx);
            }
        }
        List<Object> missing = (List<Object>) m.get("idVertexMap_missing");
        if (missing != null) {
            for (Object o2 : missing) {
                Vertex vx = g.idVertexMap.remove(((Number) o2).intValue());
                if (vx != null) g.verticeSet.remove(vx);
            }
        }
        boolean hash = "HashSet".equals(m.get("adj_type"));
        for (Object e : (List<Object>) m.get("adj")) {
            List<Object> p = (List<Object>) e;
            Set<Integer> s = hash ? new HashSet<Integer>() : new TreeSet<Integer>();
            for (Object x : (List<Object>) p.get(1)) s.add(((Number) x).intValue());
            g.adjacencySetMap.put(((Number) p.get(0)).intValue(), s);
        }
        setField(g, "rows", m.get("rows") == null ? 0 : ((Number) m.get("rows")).intValue());
        setField(g, "columns", m.get("cols") == null ? 0 : ((Number) m.get("cols")).intValue());
        return g;
    }

    static Graph G(Ctx c, Map<String, Object> a, String k) throws Exception {
        Graph g = toGraph(req(a, k));
        if (g != null) c.track(k, g);
        return g;
    }

    @SuppressWarnings("unchecked")
    static Problem toProblem(Object o) throws Exception {
        if (o == null) return null;
        Map<String, Object> m = (Map<String, Object>) o;
        Problem p = new Problem();
        p.graph = toGraph(m.get("graph"));
        p.sg = toIntArr2(m.get("sg"));
        return p;
    }

    static Problem P(Ctx c, Map<String, Object> a, String k) throws Exception {
        Problem p = toProblem(req(a, k));
        if (p != null) c.track(k, p);
        return p;
    }

    static int[][] distanceTable(Graph g, int[] goal, Object spec) {
        if (spec == null) return null;
        if ("fromGoals".equals(spec)) {
            int[][] d = new int[goal.length][g.vertices.length];
            for (int a = 0; a < goal.length; a++) PathFinder.getDistancesToAllVertices(g, goal[a], d[a]);
            return d;
        }
        return toIntArr2(spec);
    }

    // ------------------------------------------------------------------------------------
    // Result conversion (Java -> JSON)
    // ------------------------------------------------------------------------------------

    static Object num(double d) {
        if (d == Math.rint(d) && !Double.isInfinite(d) && Math.abs(d) < 1e15) return (long) d;
        return d;
    }

    @SuppressWarnings("unchecked")
    static Object toJson(Object o) {
        if (o == null) return null;
        if (o instanceof String || o instanceof Boolean || o instanceof Long || o instanceof Integer
                || o instanceof Double) return o;
        // Maps built by the harness are LinkedHashMaps with string keys -> JSON objects.
        if (o instanceof LinkedHashMap) return jsonMap((Map<String, Object>) o);
        if (o instanceof Float) return ((Float) o).doubleValue();
        if (o instanceof Short || o instanceof Byte) return ((Number) o).longValue();
        if (o instanceof int[]) {
            List<Object> l = new ArrayList<Object>();
            for (int x : (int[]) o) l.add(x);
            return l;
        }
        if (o instanceof long[]) {
            List<Object> l = new ArrayList<Object>();
            for (long x : (long[]) o) l.add(x);
            return l;
        }
        if (o instanceof double[]) {
            List<Object> l = new ArrayList<Object>();
            for (double x : (double[]) o) l.add(x);
            return l;
        }
        if (o instanceof boolean[]) {
            List<Object> l = new ArrayList<Object>();
            for (boolean x : (boolean[]) o) l.add(x);
            return l;
        }
        if (o instanceof Object[] && !(o instanceof Path[])) {
            List<Object> l = new ArrayList<Object>();
            for (Object x : (Object[]) o) l.add(toJson(x));
            return l;
        }
        if (o instanceof Path) return toJson(((Path) o).vertexList);
        if (o instanceof Path[]) {
            List<Object> l = new ArrayList<Object>();
            for (Path p : (Path[]) o) l.add(p == null ? null : toJson(p.vertexList));
            return l;
        }
        if (o instanceof Graph) return graphJson((Graph) o);
        if (o instanceof Problem) return problemJson((Problem) o);
        if (o instanceof Collection) {
            List<Object> l = new ArrayList<Object>();
            for (Object x : (Collection<?>) o) l.add(toJson(x));
            return l;
        }
        if (o instanceof Map) {
            // Non-string keys (or a plain HashMap): list of [key, value] pairs in iteration order.
            List<Object> l = new ArrayList<Object>();
            for (Map.Entry<?, ?> e : ((Map<?, ?>) o).entrySet()) l.add(Arrays.asList(toJson(e.getKey()), toJson(e.getValue())));
            return l;
        }
        if (o instanceof StringBuffer || o instanceof StringBuilder) return o.toString();
        return "<" + o.getClass().getName() + ">";
    }

    static Map<String, Object> jsonMap(Map<String, Object> m) {
        Map<String, Object> r = new LinkedHashMap<String, Object>();
        for (Map.Entry<String, Object> e : m.entrySet()) r.put(e.getKey(), toJson(e.getValue()));
        return r;
    }

    static Map<String, Object> graphJson(Graph g) {
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        try {
            if (g.vertices == null) {
                m.put("vertices", null);
            } else {
                List<Object[]> vs = new ArrayList<Object[]>();
                boolean sorted = true;
                int last = Integer.MIN_VALUE;
                for (Vertex v : g.vertices) {
                    Object[] e;
                    Point2D pt = (Point2D) getField(v, "point");
                    if (pt == null) e = new Object[]{v.id};
                    else e = new Object[]{v.id, num(pt.getX()), num(pt.getY())};
                    vs.add(e);
                    if (v.id < last) sorted = false;
                    last = v.id;
                }
                if (!sorted) {
                    // The original's order here comes from a HashSet<Vertex> with identity hash
                    // codes, i.e. it is unspecified (and varies between runs); store it sorted.
                    java.util.Collections.sort(vs, (x, y) -> Integer.compare((Integer) x[0], (Integer) y[0]));
                }
                List<Object> l = new ArrayList<Object>();
                for (Object[] e : vs) l.add(Arrays.asList(e));
                m.put("vertices", l);
            }
            List<Object> adj = new ArrayList<Object>();
            String type = null;
            for (Integer k : new TreeSet<Integer>(g.adjacencySetMap.keySet())) {
                Set<Integer> s = g.adjacencySetMap.get(k);
                String t = s == null ? "null" : s.getClass().getSimpleName();
                if (type == null) type = t;
                else if (!type.equals(t)) type = "mixed";
                adj.add(Arrays.asList(k, toJson(s)));
            }
            m.put("adj", adj);
            m.put("adj_type", type == null ? "TreeSet" : type);
            m.put("rows", getField(g, "rows"));
            m.put("cols", getField(g, "columns"));
            // Only when idVertexMap disagrees with 'vertices' (a stale array, e.g. after
            // removeVertex or addVertex without finishBuildingGraph).
            Set<Integer> vids = new TreeSet<Integer>();
            if (g.vertices != null) for (Vertex v : g.vertices) vids.add(v.id);
            List<Object> extra = new ArrayList<Object>();
            for (Map.Entry<Integer, Vertex> e : g.idVertexMap.entrySet()) {
                if (vids.contains(e.getKey())) continue;
                Point2D pt = (Point2D) getField(e.getValue(), "point");
                extra.add(pt == null ? Arrays.asList((Object) e.getKey())
                        : Arrays.asList((Object) e.getKey(), num(pt.getX()), num(pt.getY())));
            }
            List<Object> missing = new ArrayList<Object>();
            for (Integer id : vids) if (!g.idVertexMap.containsKey(id)) missing.add(id);
            if (!extra.isEmpty()) m.put("idVertexMap_extra", extra);
            if (!missing.isEmpty()) m.put("idVertexMap_missing", missing);
            if (g.verticeSet.size() != g.idVertexMap.size()) m.put("verticeSet_size", g.verticeSet.size());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        return m;
    }

    static Map<String, Object> problemJson(Problem p) {
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        m.put("graph", p.graph == null ? null : graphJson(p.graph));
        m.put("sg", toJson(p.sg));
        if (p.vidNewIdMap != null) m.put("vidNewIdMap", toJson(p.vidNewIdMap));
        if (p.newIdVidMap != null) m.put("newIdVidMap", toJson(p.newIdVidMap));
        return m;
    }

    // ------------------------------------------------------------------------------------
    // Reflection helpers
    // ------------------------------------------------------------------------------------

    static Object getField(Object o, String name) throws Exception {
        Class<?> c = o.getClass();
        while (c != null) {
            try {
                Field f = c.getDeclaredField(name);
                f.setAccessible(true);
                return f.get(o);
            } catch (NoSuchFieldException e) {
                c = c.getSuperclass();
            }
        }
        throw new NoSuchFieldException(name);
    }

    static void setField(Object o, String name, Object v) throws Exception {
        Class<?> c = o.getClass();
        while (c != null) {
            try {
                Field f = c.getDeclaredField(name);
                f.setAccessible(true);
                f.set(o, v);
                return;
            } catch (NoSuchFieldException e) {
                c = c.getSuperclass();
            }
        }
        throw new NoSuchFieldException(name);
    }

    static Object call(Object target, Class<?> c, String name, Class<?>[] types, Object... vals) throws Throwable {
        Method m = c.getDeclaredMethod(name, types);
        m.setAccessible(true);
        try {
            return m.invoke(target, vals);
        } catch (InvocationTargetException e) {
            throw e.getCause();
        }
    }

    /** Invoke a uniquely named method, converting JSON arguments by parameter type. */
    static Object callByName(Object target, Class<?> c, String name, List<Object> args) throws Throwable {
        Method found = null;
        for (Method m : c.getDeclaredMethods()) {
            if (m.getName().equals(name) && m.getParameterTypes().length == args.size()) {
                if (found != null) throw new IllegalArgumentException("ambiguous " + name);
                found = m;
            }
        }
        if (found == null) throw new NoSuchMethodException(name);
        Class<?>[] ts = found.getParameterTypes();
        Object[] vals = new Object[ts.length];
        for (int i = 0; i < ts.length; i++) {
            Object a = args.get(i);
            if (ts[i] == int.class) vals[i] = ((Number) a).intValue();
            else if (ts[i] == long.class) vals[i] = ((Number) a).longValue();
            else if (ts[i] == double.class) vals[i] = toDouble(a);
            else if (ts[i] == boolean.class) vals[i] = (Boolean) a;
            else if (ts[i] == String.class) vals[i] = subst((String) a);
            else if (ts[i] == String[].class) {
                List<?> l = (List<?>) a;
                String[] s = new String[l.size()];
                for (int k = 0; k < s.length; k++) s[k] = subst((String) l.get(k));
                vals[i] = s;
            } else throw new IllegalArgumentException("unsupported parameter type " + ts[i]);
        }
        found.setAccessible(true);
        try {
            return found.invoke(target, vals);
        } catch (InvocationTargetException e) {
            throw e.getCause();
        }
    }

    static final Class<?> GR = Graph.class, IAC = int[].class, IA2C = int[][].class;
    static final Class<?> IN = int.class, DB = double.class, BO = boolean.class;

    // ------------------------------------------------------------------------------------
    // Model dumps (canonical text, see README.md)
    // ------------------------------------------------------------------------------------

    static String fmt(double d) {
        if (Double.isNaN(d)) return "nan";
        if (d >= 1e100) return "inf";
        if (d <= -1e100) return "-inf";
        if (d == Math.rint(d) && Math.abs(d) < 1e15) return Long.toString((long) d);
        return Double.toString(d);
    }

    // The original's key arithmetic, copied verbatim (int sub-expressions wrap).
    static long edgeKey(int a, int v1, int v2, int t) {
        return a * 1000000000000L + v1 * 100000000 + v2 * 10000 + t;
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> dumpModel(GRBModel model, Object solver, Graph graph, int[] starts, int[] goals,
                                         int T) throws Exception {
        model.update();
        SortedMap<Long, GRBVar> evm = (SortedMap<Long, GRBVar>) getField(solver, "edgeVarMap");
        Map<Long, String> keyName = new HashMap<Long, String>();
        int n = graph.vertices.length;
        for (int a = 0; a < starts.length; a++) {
            for (int t = 0; t < T; t++) {
                for (int v1 = 0; v1 < n; v1++) {
                    Set<Integer> adj = graph.adjacencySetMap.get(v1);
                    if (adj == null) continue;
                    for (int v2 : adj) {
                        long k = edgeKey(a, v1, v2, t);
                        if (!evm.containsKey(k)) continue;
                        String nm = "x[" + a + "," + v1 + "," + v2 + "," + t + "]";
                        String old = keyName.get(k);
                        keyName.put(k, old == null ? nm : old + "|" + nm);
                    }
                }
            }
            long k = edgeKey(a, goals[a], starts[a], T);
            String nm = "x[" + a + "," + goals[a] + "," + starts[a] + "," + T + "]";
            String old = keyName.get(k);
            keyName.put(k, old == null ? nm : old + "|" + nm);
        }
        Map<Integer, String> colName = new HashMap<Integer, String>();
        for (Map.Entry<Long, GRBVar> e : evm.entrySet()) {
            String nm = keyName.get(e.getKey());
            if (nm == null) nm = "?key" + e.getKey();
            int idx = e.getValue().index();
            String old = colName.get(idx);
            colName.put(idx, old == null ? nm : old + "|" + nm);
        }
        // Total-time auxiliaries.
        try {
            GRBVar[] yT = (GRBVar[]) getField(solver, "yTVars");
            GRBVar[][] y = (GRBVar[][]) getField(solver, "yVars");
            if (yT != null) for (int a = 0; a < yT.length; a++) if (yT[a] != null) colName.put(yT[a].index(), "yT[" + a + "]");
            if (y != null)
                for (int a = 0; a < y.length; a++)
                    for (int t = 0; t < y[a].length; t++) if (y[a][t] != null) colName.put(y[a][t].index(), "y[" + a + "," + t + "]");
        } catch (NoSuchFieldException e) {
            // not a total-time model
        }

        StringBuilder sb = new StringBuilder();
        GRBVar[] vars = model.getVars();
        GRBConstr[] cons = model.getConstrs();
        int sense = model.get(GRB.IntAttr.ModelSense);
        sb.append("SENSE ").append(sense == GRB.MAXIMIZE ? "max" : "min").append('\n');
        sb.append("NCOLS ").append(vars.length).append('\n');
        sb.append("NROWS ").append(cons.length).append('\n');
        for (int i = 0; i < vars.length; i++) {
            GRBVar v = vars[i];
            char vt = v.get(GRB.CharAttr.VType);
            String nm = colName.get(i);
            if (nm == null) nm = "?" + i;
            sb.append('C').append(i).append(' ').append(fmt(v.get(GRB.DoubleAttr.LB))).append(' ')
              .append(fmt(v.get(GRB.DoubleAttr.UB))).append(' ').append(vt).append(' ')
              .append(fmt(v.get(GRB.DoubleAttr.Obj))).append(' ').append(nm).append('\n');
        }
        long nnz = 0;
        for (int i = 0; i < cons.length; i++) {
            GRBLinExpr row = model.getRow(cons[i]);
            TreeMap<Integer, Double> terms = new TreeMap<Integer, Double>();
            for (int j = 0; j < row.size(); j++) {
                int idx = row.getVar(j).index();
                Double old = terms.get(idx);
                terms.put(idx, (old == null ? 0.0 : old) + row.getCoeff(j));
            }
            char s = cons[i].get(GRB.CharAttr.Sense);
            String ss = s == GRB.LESS_EQUAL ? "<=" : s == GRB.GREATER_EQUAL ? ">=" : "=";
            sb.append('R').append(i).append(' ').append(ss).append(' ').append(fmt(cons[i].get(GRB.DoubleAttr.RHS))).append(" :");
            for (Map.Entry<Integer, Double> e : terms.entrySet()) {
                if (e.getValue() == 0.0) continue;
                sb.append(" (").append(e.getKey()).append(',').append(fmt(e.getValue())).append(')');
                nnz++;
            }
            sb.append('\n');
        }
        String text = sb.toString();
        Map<String, Object> m = new LinkedHashMap<String, Object>();
        m.put("ncols", vars.length);
        m.put("nrows", cons.length);
        m.put("nnz", nnz);
        m.put("sense", sense == GRB.MAXIMIZE ? "max" : "min");
        m.put("edgeVarMap_size", evm.size());
        m.put("sha256", sha256(text.getBytes(StandardCharsets.UTF_8)));
        if (text.length() <= MAX_DUMP) m.put("dump", text);
        Map<String, Object> params = new LinkedHashMap<String, Object>();
        params.put("TimeLimit", model.get(GRB.DoubleParam.TimeLimit));
        params.put("MIPGap", model.get(GRB.DoubleParam.MIPGap));
        params.put("OutputFlag", model.get(GRB.IntParam.OutputFlag));
        params.put("Threads", model.get(GRB.IntParam.Threads));
        m.put("params", params);
        return m;
    }

    static Object modelCase(String fn, Map<String, Object> a, Ctx c) throws Throwable {
        String clsName = fn.substring(0, fn.indexOf('.'));
        Graph g = G(c, a, "graph");
        int[] s = IA(c, a, "starts");
        int[] gl = IA(c, a, "goals");
        int T = I(a, "timeSteps");
        boolean integer = B(a, "integer");
        GRBEnv env = new GRBEnv();
        env.set(GRB.IntParam.OutputFlag, 0);
        if (a.containsKey("gap")) env.set(GRB.DoubleParam.MIPGap, D(a, "gap"));
        Object solver;
        GRBModel model;
        if (clsName.equals("MultiagentGraphSolverGurobiTimePuzzle")) {
            PuzzleSolver ps = new PuzzleSolver(g, s, gl);
            call(ps, PuzzleSolver.class, "updateReachableSets", new Class<?>[]{GR, IAC, IAC, IN}, g, s, gl, T);
            boolean[][][] rv = (boolean[][][]) getField(ps, "reachableVertices");
            solver = new MultiagentGraphSolverGurobiTimePuzzle();
            model = (GRBModel) call(solver, solver.getClass(), "prepareModel",
                    new Class<?>[]{GRBEnv.class, GR, IAC, IAC, BO, BO, IN, boolean[][][].class},
                    env, g, s, gl, integer, B(a, "setOptimal"), T, rv);
        } else {
            solver = cls(clsName).getDeclaredConstructor().newInstance();
            call(solver, solver.getClass(), "updateReachableSets", new Class<?>[]{GR, IAC, IAC, IN}, g, s, gl, T);
            if (clsName.equals("MultiagentGraphSolverGurobiTotalTime") || clsName.equals("MultiagentGraphSolverGurobiDistance")) {
                model = (GRBModel) call(solver, solver.getClass(), "prepareModel",
                        new Class<?>[]{GRBEnv.class, GR, IAC, IAC, BO, IN, DB},
                        env, g, s, gl, integer, T, D(a, "timeLimit"));
            } else {
                model = (GRBModel) call(solver, solver.getClass(), "prepareModel",
                        new Class<?>[]{GRBEnv.class, GR, IAC, IAC, BO, BO, IN, DB},
                        env, g, s, gl, integer, B(a, "setOptimal"), T, D(a, "timeLimit"));
            }
        }
        Map<String, Object> d = dumpModel(model, solver, g, s, gl, T);
        model.dispose();
        env.dispose();
        return d;
    }

    static Object reachableCase(String clsName, Map<String, Object> a, Ctx c) throws Throwable {
        Graph g = G(c, a, "graph");
        int[] s = IA(c, a, "starts");
        int[] gl = IA(c, a, "goals");
        int T = I(a, "timeSteps");
        Object solver = clsName.equals("PuzzleSolver") ? new PuzzleSolver(g, s, gl)
                : cls(clsName).getDeclaredConstructor().newInstance();
        call(solver, solver.getClass(), "updateReachableSets", new Class<?>[]{GR, IAC, IAC, IN}, g, s, gl, T);
        boolean[][][] rv = (boolean[][][]) getField(solver, "reachableVertices");
        List<Object> out = new ArrayList<Object>();
        for (boolean[][] perAgent : rv) {
            List<Object> byT = new ArrayList<Object>();
            for (int t = 0; t <= T; t++) {
                List<Object> vs = new ArrayList<Object>();
                for (int v = 0; v < perAgent.length; v++) if (perAgent[v][t]) vs.add(v);
                byT.add(vs);
            }
            out.add(byT);
        }
        return out;
    }

    @SuppressWarnings("unchecked")
    static Object keyHelpers(String clsName, Map<String, Object> a) throws Throwable {
        Object solver = cls(clsName).getDeclaredConstructor().newInstance();
        Class<?> k = solver.getClass();
        List<Object> out = new ArrayList<Object>();
        for (Object o : (List<Object>) req(a, "tuples")) {
            List<Object> tp = (List<Object>) o;
            int ag = ((Number) tp.get(0)).intValue(), v1 = ((Number) tp.get(1)).intValue();
            int v2 = ((Number) tp.get(2)).intValue(), t = ((Number) tp.get(3)).intValue();
            Map<String, Object> m = new LinkedHashMap<String, Object>();
            long e = (Long) call(solver, k, "getEdgeId", new Class<?>[]{IN, IN, IN, IN}, ag, v1, v2, t);
            m.put("edgeId", e);
            m.put("vertexTimeId", call(solver, k, "getVertexTimeId", new Class<?>[]{IN, IN}, v1, t));
            m.put("agentVertexTimeId", call(solver, k, "getAgentVertexTimeId", new Class<?>[]{IN, IN, IN}, ag, v1, t));
            m.put("edgeTimeId", call(solver, k, "getEdgeTimeId", new Class<?>[]{IN, IN, IN}, v1, v2, t));
            if (clsName.equals("MultiagentGraphSolverGurobiTimeR")) {
                m.put("timeFromEdgeId", call(solver, k, "getTimeFromEdgeId", new Class<?>[]{long.class}, e));
                m.put("isV1V2Same", call(solver, k, "isV1V2Same", new Class<?>[]{long.class}, e));
            }
            out.add(m);
        }
        return out;
    }

    // ------------------------------------------------------------------------------------
    // Dispatch
    // ------------------------------------------------------------------------------------

    static List<Object> statNoTime(long[] s, int timeIndex) {
        if (s == null) return null;
        List<Object> l = new ArrayList<Object>();
        for (int i = 0; i < s.length; i++) l.add(i == timeIndex ? null : (Object) s[i]);
        return l;
    }

    static final Class<?> MAIN = Main.class;
    static final Class<?> PRB = Problem.class;

    @SuppressWarnings("unchecked")
    static Object dispatch(String fn, Map<String, Object> a, Ctx c) throws Throwable {
        Object r = dispatch0(fn, a, c);
        c.mutated.putAll(c.diff());
        return r;
    }

    @SuppressWarnings("unchecked")
    static Object dispatch0(String fn, Map<String, Object> a, Ctx c) throws Throwable {
        if (fn.endsWith(".prepareModel")) return modelCase(fn, a, c);
        if (fn.endsWith(".updateReachableSets")) return reachableCase(fn.substring(0, fn.indexOf('.')), a, c);
        if (fn.endsWith(".keyHelpers")) return keyHelpers(fn.substring(0, fn.indexOf('.')), a);
        if (fn.startsWith("Main.run")) {
            List<Object> l = a.containsKey("params") ? (List<Object>) a.get("params") : new ArrayList<Object>();
            callByName(null, MAIN, fn.substring(5), l);
            return null;
        }
        switch (fn) {
            case "META": {
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("gurobi_version", GRB.VERSION_MAJOR + "." + GRB.VERSION_MINOR + "." + GRB.VERSION_TECHNICAL);
                m.put("java_version", System.getProperty("java.version"));
                m.put("java_vendor", System.getProperty("java.vendor"));
                m.put("java_vm", System.getProperty("java.vm.name") + " " + System.getProperty("java.vm.version"));
                m.put("os", System.getProperty("os.name") + " " + System.getProperty("os.arch"));
                GRBEnv env = new GRBEnv();
                m.put("gurobi_threads_param", env.get(GRB.IntParam.Threads));
                env.dispose();
                return m;
            }

            // ---------------- Graph / Vertex / Path ----------------
            case "Graph.create2DGridGraph":
                return Graph.create2DGridGraph(I(a, "rows"), I(a, "cols"), B(a, "update"));
            case "Graph.create2DGridGraph8Connected":
                return Graph.create2DGridGraph8Connected(I(a, "rows"), I(a, "cols"), B(a, "update"));
            case "Graph.createGeneric2DGridGraphWithHoles":
                return Graph.createGeneric2DGridGraphWithHoles(I(a, "rows"), I(a, "cols"), D(a, "percentObstacle"));
            case "Graph.createGeneric2DGridGraphWithHoles8Connected":
                return Graph.createGeneric2DGridGraphWithHoles8Connected(I(a, "rows"), I(a, "cols"), D(a, "percentObstacle"));
            case "Graph.getRandomStartGoal":
                return Graph.getRandomStartGoal(G(c, a, "g"), I(a, "number"));
            case "Graph.getRandomStartGoalMany":
                return Graph.getRandomStartGoalMany(G(c, a, "g"), I(a, "number"));
            case "Graph.convertGraph": {
                Graph g = G(c, a, "graph");
                int[] s = IA(c, a, "start"), gl = IA(c, a, "goal");
                return call(null, GR, "convertGraph", new Class<?>[]{GR, IAC, IAC}, g, s, gl);
            }
            case "Graph.manual": {
                // addVertex(int, int[]) for each entry, then finishBuildingGraph() (Main case 3 style).
                Graph g = new Graph();
                for (Object o : (List<Object>) req(a, "neighbors")) {
                    List<Object> p = (List<Object>) o;
                    g.addVertex(((Number) p.get(0)).intValue(), toIntArr(p.get(1)));
                }
                g.finishBuildingGraph();
                return g;
            }
            case "Graph.getEdgeSet":
                return G(c, a, "g").getEdgeSet();
            case "Graph.getEdgeId": {
                List<Object> out = new ArrayList<Object>();
                for (Object o : (List<Object>) req(a, "pairs")) {
                    int[] p = toIntArr(o);
                    long e = Graph.getEdgeId(p[0], p[1]);
                    out.add(Arrays.asList(e, Graph.getFirstVertexOfEdge(e), Graph.getSecondVertexOfEdge(e)));
                }
                return out;
            }
            case "Graph.removeVertex(Vertex)": {
                Graph g = G(c, a, "g");
                g.removeVertex(g.idVertexMap.get(I(a, "vId")));
                return null;
            }
            case "Graph.removeVertex(int,int)": {
                Graph g = G(c, a, "g");
                call(g, GR, "removeVertex", new Class<?>[]{IN, IN}, I(a, "x"), I(a, "y"));
                return null;
            }
            case "Graph.addVertex(Vertex)": {
                Graph g = G(c, a, "g");
                Vertex v = a.get("x") == null ? new Vertex(I(a, "id"))
                        : new Vertex(new Point2D.Double(D(a, "x"), D(a, "y")), I(a, "id"));
                g.addVertex(v);
                return null;
            }
            case "Graph.addVertex(int,int[])": {
                Graph g = G(c, a, "g");
                g.addVertex(I(a, "vId"), toIntArr(req(a, "neighbors")));
                return null;
            }
            case "Graph.finishBuildingGraph": {
                Graph g = G(c, a, "g");
                g.finishBuildingGraph();
                return null;
            }
            case "Graph.getId": {
                Graph g = G(c, a, "g");
                return call(g, GR, "getId", new Class<?>[]{IN, IN}, I(a, "x"), I(a, "y"));
            }
            case "Graph.updateAdjList": {
                Graph g = G(c, a, "g");
                Graph.updateAdjList(g);
                return null;
            }
            case "Graph.isConnectedWithoutVertex":
                return G(c, a, "g").isConnectedWithoutVertex(I(a, "vId"));
            case "Graph.areVerticesConnectedWithoutV3": {
                Graph g = G(c, a, "g");
                return call(g, GR, "areVerticesConnectedWithoutV3", new Class<?>[]{IN, IN, IN},
                        I(a, "vId1"), I(a, "vId2"), I(a, "vId3"));
            }
            case "Graph.getv4TestGraph":
                return call(null, GR, "getv4TestGraph", new Class<?>[]{});
            case "Graph.getTestGraphForBridge":
                return Graph.getTestGraphForBridge();
            case "Graph.printGraphAsAdjacenyList":
                G(c, a, "g").printGraphAsAdjacenyList();
                return null;
            case "Vertex.print": {
                Graph g = G(c, a, "g");
                g.idVertexMap.get(I(a, "vId")).print();
                return null;
            }
            case "Vertex.printVertex": {
                Graph g = G(c, a, "g");
                g.idVertexMap.get(I(a, "vId")).printVertex();
                return null;
            }
            case "Vertex.printVertex(StringBuffer)": {
                Graph g = G(c, a, "g");
                StringBuffer sb = new StringBuffer();
                g.idVertexMap.get(I(a, "vId")).printVertex(sb);
                return sb.toString();
            }
            case "Path.printPath": {
                Graph g = G(c, a, "g");
                Path p = new Path();
                for (int v : toIntArr(req(a, "path"))) p.addVertex(v);
                p.printPath(g);
                return null;
            }

            // ---------------- Problem ----------------
            case "Problem.createGridProblem":
                return Problem.createGridProblem(I(a, "col"), I(a, "row"), D(a, "fracObs"), I(a, "n"));
            case "Problem.createGridProblem8Connected":
                return Problem.createGridProblem8Connected(I(a, "col"), I(a, "row"), D(a, "fracObs"), I(a, "n"));
            case "Problem.createN2Puzzle":
                return Problem.createN2Puzzle(I(a, "n"));
            case "Problem.createN2M1Puzzle":
                return Problem.createN2M1Puzzle(I(a, "n"));
            case "Problem.getA9Puzzle":
                return call(null, PRB, "getA9Puzzle", new Class<?>[]{});
            case "Problem.getLongStraightWithOneGarageProblem":
                return Problem.getLongStraightWithOneGarageProblem();
            case "Problem.writeToFile(String)": {
                Problem p = P(c, a, "p");
                File f = new File(c.dir, "out/problem.txt");
                call(p, PRB, "writeToFile", new Class<?>[]{String.class}, f.getPath());
                return null; // the file is reported under "files"
            }
            case "Problem.writeToFile(Problem,String)": {
                Problem p = P(c, a, "p");
                File f = new File(c.dir, "problem.dat");
                Problem.writeToFile(p, f.getPath());
                return null; // the file is reported under "files"
            }
            case "Problem.readFromFile": {
                String src = S(a, "source");
                File f = new File(c.dir, "in.dat");
                if (src.equals("base64")) Files.write(f.toPath(), Base64.getDecoder().decode(S(a, "data")));
                else if (src.equals("text")) Files.write(f.toPath(), S(a, "data").getBytes(StandardCharsets.UTF_8));
                else if (!src.equals("missing")) throw new IllegalArgumentException("source " + src);
                c.skipFiles.add("in.dat");
                return Problem.readFromFile(f.getPath());
            }
            case "Problem.roundTrip": {
                // Problem.writeToFile(p, f) followed by Problem.readFromFile(f).
                Problem p = P(c, a, "p");
                File f = new File(c.dir, "rt.dat");
                Problem.writeToFile(p, f.getPath());
                c.skipFiles.add("rt.dat");
                return Problem.readFromFile(f.getPath());
            }
            case "Problem.createN2Puzzles":
            case "Problem.createCrowdedHardProblems":
            case "Problem.create32x32Problems":
            case "Problem.create32x32Problems8Connected":
            case "Problem.create24x18PerformanceTestingProblems":
            case "Problem.createCrowdedHardProblems8Connected":
            case "Problem.main": {
                String folder = (String) a.get("folder");
                File out = new File(c.dir, "batch");
                if ("case".equals(folder)) {
                    out.mkdirs();
                    staticField("Problem.FILE_FOLDER").set(null, out.getAbsolutePath() + "/");
                }
                String m = fn.substring("Problem.".length());
                if (m.equals("main")) {
                    List<Object> argvl = a.containsKey("argv") ? (List<Object>) a.get("argv") : new ArrayList<Object>();
                    String[] argv = new String[argvl.size()];
                    for (int i = 0; i < argv.length; i++) argv[i] = ((String) argvl.get(i)).replace("$BATCH", out.getAbsolutePath() + "/");
                    out.mkdirs();
                    Problem.main(argv);
                } else {
                    call(null, PRB, m, new Class<?>[]{});
                }
                Map<String, Object> d = batchDigest(out, a.containsKey("samples") ? I(a, "samples") : 1);
                deleteRecursively(out);
                return d;
            }

            // ---------------- PathFinder ----------------
            case "PathFinder.getDistancesToAllVertices": {
                Graph g = G(c, a, "g");
                int[] in = IA(c, a, "inputDistances");
                return PathFinder.getDistancesToAllVertices(g, I(a, "start"), in);
            }
            case "PathFinder.findGoalsInDistanceRange":
                return PathFinder.findGoalsInDistanceRange(G(c, a, "g"), I(a, "start"), I(a, "dMin"), I(a, "dMax"));
            case "PathFinder.findShortestPaths": {
                Graph g = G(c, a, "g");
                return PathFinder.findShortestPaths(g, IA(c, a, "start"), IA(c, a, "goal"));
            }
            case "PathFinder.findOneShortestPath": {
                Graph g = G(c, a, "g");
                return PathFinder.findOneShortestPath(g, I(a, "start"), I(a, "goal"), SET(c, a, "alreadyVisitedVertices"));
            }
            case "PathFinder.getTotalDistance(int[][])":
                return PathFinder.getTotalDistance(IA2(c, a, "paths"));
            case "PathFinder.getTotalDistance(Path[])":
                return PathFinder.getTotalDistance(PA(c, a, "paths"));
            case "PathFinder.getMakespanLowerBound": {
                Graph g = G(c, a, "g");
                return PathFinder.getMakespanLowerBound(g, IA(c, a, "start"), IA(c, a, "goal"));
            }
            case "PathFinder.getTotalTimeLowerBound": {
                Graph g = G(c, a, "g");
                return PathFinder.getTotalTimeLowerBound(g, IA(c, a, "start"), IA(c, a, "goal"));
            }

            // ---------------- Solvers ----------------
            case "MultiagentGraphSolverGurobiTime.solve": {
                Graph g = G(c, a, "graph");
                return new MultiagentGraphSolverGurobiTime().solve(g, IA(c, a, "starts"), IA(c, a, "goals"),
                        B(a, "solveRelaxedFirst"), B(a, "setOptimal"), I(a, "extraSteps"), B(a, "hasTimeLimit"), D(a, "timeLimit"));
            }
            case "MultiagentGraphSolverGurobiTimeR.solve": {
                Graph g = G(c, a, "graph");
                return new MultiagentGraphSolverGurobiTimeR().solve(g, IA(c, a, "starts"), IA(c, a, "goals"),
                        B(a, "solveRelaxedFirst"), B(a, "setOptimal"), I(a, "extraSteps"), B(a, "hasTimeLimit"), D(a, "timeLimit"));
            }
            case "MultiagentGraphSolverGurobiTotalTime.solve": {
                Graph g = G(c, a, "graph");
                return new MultiagentGraphSolverGurobiTotalTime().solve(g, IA(c, a, "starts"), IA(c, a, "goals"),
                        IA(c, a, "opt"), D(a, "gap"), I(a, "timeSteps"), B(a, "hasTimeLimit"), D(a, "timeLimit"));
            }
            case "MultiagentGraphSolverGurobiDistance.solve": {
                Graph g = G(c, a, "graph");
                return new MultiagentGraphSolverGurobiDistance().solve(g, IA(c, a, "starts"), IA(c, a, "goals"),
                        D(a, "gap"), I(a, "expansion"), D(a, "timeLimit"));
            }
            case "PuzzleSolver.solve(double)": {
                Graph g = G(c, a, "graph");
                PuzzleSolver ps = new PuzzleSolver(g, IA(c, a, "starts"), IA(c, a, "goals"));
                return ps.solve(D(a, "timeLimit"));
            }
            case "PuzzleSolver.solve(Problem,double)":
                return PuzzleSolver.solve(P(c, a, "p"), D(a, "timeLimit"));

            // ---------------- PathPlanner (static utilities) ----------------
            case "PathPlanner.printPaths":
                PathPlanner.printPaths(G(c, a, "g"), IA2(c, a, "paths"));
                return null;
            case "PathPlanner.printPathToString":
                return PathPlanner.printPathToString(G(c, a, "g"), IA2(c, a, "paths"));
            case "PathPlanner.printStartAndGoals": {
                Graph g = G(c, a, "g");
                PathPlanner.printStartAndGoals(g, IA(c, a, "start"), IA(c, a, "goal"));
                return null;
            }
            case "PathPlanner.numberOfGoalsReached":
                return PathPlanner.numberOfGoalsReached(IA2(c, a, "paths"), IA(c, a, "goal"));
            case "PathPlanner.parsePaths":
                return PathPlanner.parsePaths(PA(c, a, "paths"));
            case "PathPlanner.hasCycles":
                return PathPlanner.hasCycles(IA2(c, a, "paths"));
            case "PathPlanner.isPathSetValid": {
                int[][] p = IA2(c, a, "paths");
                Graph g = G(c, a, "graph");
                return PathPlanner.isPathSetValid(p, g, IA(c, a, "starts"), IA(c, a, "goals"));
            }
            case "PathPlanner.mergePaths": {
                int[][] x = IA2(c, a, "pathFromStart"), y = IA2(c, a, "pathFromMiddle");
                return call(c.pp(), PathPlanner.class, "mergePaths", new Class<?>[]{IA2C, IA2C}, x, y);
            }
            case "PathPlanner.moveAgentsIndependently":
                return c.pp().moveAgentsIndependently(IA2(c, a, "paths"));
            case "PathPlanner.moveAgentsIndependentlyWhenCan": {
                Graph g = G(c, a, "g");
                int[] s = IA(c, a, "start"), gl = IA(c, a, "goal");
                Path[] pa = PA(c, a, "pathArray");
                return call(c.pp(), PathPlanner.class, "moveAgentsIndependentlyWhenCan",
                        new Class<?>[]{GR, IAC, IAC, Path[].class}, g, s, gl, pa);
            }
            case "PathPlanner.createSubProblem": {
                PathPlanner pp = c.pp();
                Graph g = G(c, a, "g");
                int[][] paths = IA2(c, a, "paths");
                int[] goal = IA(c, a, "goal");
                if (a.containsKey("attempt")) pp.attempt = I(a, "attempt");
                if (a.containsKey("distanceToGoals")) pp.distanceToGoals = distanceTable(g, goal, a.get("distanceToGoals"));
                return call(pp, PathPlanner.class, "createSubProblem",
                        new Class<?>[]{GR, IA2C, Integer[].class, IAC, IN}, g, paths, IGA(a, "svs"), goal, I(a, "distance"));
            }
            case "PathPlanner.splitPaths": {
                Graph g = G(c, a, "g");
                int[] s = IA(c, a, "start"), gl = IA(c, a, "goal");
                return call(c.pp(), PathPlanner.class, "splitPaths", new Class<?>[]{GR, IAC, IAC, BO}, g, s, gl, B(a, "random"));
            }
            case "PathPlanner.splitPathBySize": {
                Graph g = G(c, a, "g");
                int[] s = IA(c, a, "start"), gl = IA(c, a, "goal");
                return call(c.pp(), PathPlanner.class, "splitPathBySize", new Class<?>[]{GR, IAC, IAC, BO, IN},
                        g, s, gl, B(a, "random"), I(a, "firstSize"));
            }
            case "PathPlanner.getLongestPathLength": {
                Graph g = G(c, a, "g");
                int[] s = IA(c, a, "start"), gl = IA(c, a, "goal");
                return call(c.pp(), PathPlanner.class, "getLongestPathLength", new Class<?>[]{GR, IAC, IAC}, g, s, gl);
            }

            // ---------------- PathPlanner (planners) ----------------
            case "PathPlanner.planPathsAdvanced": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvanced(g, IA(c, a, "start"), IA(c, a, "goal"), B(a, "solveRelaxed"),
                        I(a, "extraSteps"), B(a, "setOptimal"), D(a, "timeLimit"));
            }
            case "PathPlanner.planPathsAdvancedPMG": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedPMG(g, IA(c, a, "start"), IA(c, a, "goal"), B(a, "solveRelaxed"),
                        I(a, "extraSteps"), B(a, "setOptimal"), D(a, "timeLimit"));
            }
            case "PathPlanner.planPathsAdvancedTT": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedTT(g, IA(c, a, "start"), IA(c, a, "goal"), IA(c, a, "opt"),
                        D(a, "gap"), I(a, "timeSteps"), D(a, "timeLimit"));
            }
            case "PathPlanner.planPathsAdvancedTD": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedTD(g, IA(c, a, "start"), IA(c, a, "goal"), D(a, "gap"),
                        I(a, "timeSteps"), D(a, "timeLimit"));
            }
            case "PathPlanner.planPathsAdvancedSplit": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedSplit(g, IA(c, a, "start"), IA(c, a, "goal"), B(a, "setOptimal"),
                        D(a, "timeLimit"), I(a, "splits"));
            }
            case "PathPlanner.planPathsAdvancedSplitDist": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedSplitDist(g, IA(c, a, "start"), IA(c, a, "goal"), D(a, "gap"),
                        D(a, "timeLimit"), I(a, "splits"));
            }
            case "PathPlanner.planPathsAdvancedSplitNoCycle": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedSplitNoCycle(g, IA(c, a, "start"), IA(c, a, "goal"), B(a, "setOptimal"),
                        D(a, "timeLimit"), I(a, "splits"), B(a, "top"));
            }
            case "PathPlanner.planPathsAdvancedSplitBySize": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedSplitBySize(g, IA(c, a, "start"), IA(c, a, "goal"), B(a, "setOptimal"),
                        D(a, "timeLimit"), I(a, "size"));
            }
            case "PathPlanner.planPathsAdvancedSplitBySizeNoCycle": {
                Graph g = G(c, a, "g");
                return c.pp().planPathsAdvancedSplitBySizeNoCycle(g, IA(c, a, "start"), IA(c, a, "goal"),
                        B(a, "setOptimal"), D(a, "timeLimit"), I(a, "size"), B(a, "top"));
            }
            case "PathPlanner.multiThreadedSplitPlanning": {
                Graph g = G(c, a, "g");
                PathPlanner pp = c.pp();
                int[][] r = pp.multiThreadedSplitPlanning(g, IA(c, a, "start"), IA(c, a, "goal"), D(a, "timeLimit"), I(a, "splits"));
                c.extra.put("msg", toJson(pp.msg));
                c.extra.put("mpath_keys", toJson(new TreeSet<Integer>(pp.mpath.keySet())));
                return r;
            }
            case "PathPlanner.planHeuristicPaths": {
                Graph g = G(c, a, "graph");
                return c.pp().planHeuristicPaths(g, IA(c, a, "start"), IA(c, a, "goal"));
            }
            case "PathPlanner.planHeuristicPath": {
                Graph g = G(c, a, "g");
                PathPlanner pp = c.pp();
                int[] goal = IA(c, a, "goal");
                if (a.containsKey("attempt")) pp.attempt = I(a, "attempt");
                if (a.containsKey("distanceToGoals")) pp.distanceToGoals = distanceTable(g, goal, a.get("distanceToGoals"));
                return pp.planHeuristicPath(g, IA(c, a, "start"), goal, D(a, "timeLimit"));
            }
            case "PathPlanner.main":
                PathPlanner.main(new String[0]);
                return null;

            // ---------------- HeuristicPathPlanner ----------------
            case "HeuristicPathPlanner.printPaths":
                HeuristicPathPlanner.printPaths(G(c, a, "g"), IA2(c, a, "paths"));
                return null;
            case "HeuristicPathPlanner.numberOfGoalsReached":
                return HeuristicPathPlanner.numberOfGoalsReached(IA2(c, a, "paths"), IA(c, a, "goal"));
            case "HeuristicPathPlanner.planHeuristicPaths": {
                Graph g = G(c, a, "graph");
                return c.hpp().planHeuristicPaths(g, IA(c, a, "start"), IA(c, a, "goal"));
            }
            case "HeuristicPathPlanner.planHeuristicPath": {
                Graph g = G(c, a, "g");
                HeuristicPathPlanner hp = c.hpp();
                int[] goal = IA(c, a, "goal");
                if (a.containsKey("distanceToGoals")) hp.distanceToGoals = distanceTable(g, goal, a.get("distanceToGoals"));
                return hp.planHeuristicPath(g, IA(c, a, "s"), goal, D(a, "timeLimit"));
            }
            case "HeuristicPathPlanner.existsEdgeCollision":
                return call(c.hpp(), HeuristicPathPlanner.class, "existsEdgeCollision", new Class<?>[]{IA2C, IN},
                        IA2(c, a, "paths"), I(a, "t"));
            case "HeuristicPathPlanner.existsVertexCollision":
                return call(c.hpp(), HeuristicPathPlanner.class, "existsVertexCollision", new Class<?>[]{IA2C, IN},
                        IA2(c, a, "paths"), I(a, "t"));
            case "HeuristicPathPlanner.moveAgentsIndependentlyOneStep": {
                Path[] pa = PA(c, a, "pathArray");
                return call(c.hpp(), HeuristicPathPlanner.class, "moveAgentsIndependentlyOneStep",
                        new Class<?>[]{Path[].class, IA2C, IN}, pa, IA2(c, a, "paths"), I(a, "t"));
            }
            case "HeuristicPathPlanner.createSubProblem": {
                HeuristicPathPlanner hp = c.hpp();
                Graph g = G(c, a, "g");
                int[][] paths = IA2(c, a, "paths");
                int[] goal = IA(c, a, "goal");
                if (a.containsKey("distanceToGoals")) hp.distanceToGoals = distanceTable(g, goal, a.get("distanceToGoals"));
                return call(hp, HeuristicPathPlanner.class, "createSubProblem",
                        new Class<?>[]{GR, IA2C, IN, Integer[].class, IAC, IN}, g, paths, I(a, "ct"), IGA(a, "svs"), goal, I(a, "distance"));
            }
            case "HeuristicPathPlanner.main":
                HeuristicPathPlanner.main(new String[0]);
                return null;

            // ---------------- Several calls on one planner instance ----------------
            case "Sequence": {
                c.shared = true;
                List<Object> out = new ArrayList<Object>();
                for (Object o : (List<Object>) req(a, "calls")) {
                    Map<String, Object> call = (Map<String, Object>) o;
                    Map<String, Object> r = new LinkedHashMap<String, Object>();
                    r.put("fn", call.get("fn"));
                    try {
                        r.put("result", toJson(dispatch0((String) call.get("fn"), (Map<String, Object>) call.get("args"), c)));
                    } catch (Throwable e) {
                        r.put("exception", exceptionJson(unwrap(e)));
                    }
                    Map<String, Object> mu = c.diff();
                    if (!mu.isEmpty()) r.put("mutated", mu);
                    if (!c.extra.isEmpty()) {
                        r.put("extra", new LinkedHashMap<String, Object>(c.extra));
                        c.extra.clear();
                    }
                    out.add(r);
                }
                return out;
            }

            // ---------------- Main ----------------
            case "Main.main": {
                List<Object> l = (List<Object>) req(a, "argv");
                String[] argv = new String[l.size()];
                for (int i = 0; i < argv.length; i++) argv[i] = subst((String) l.get(i));
                Main.main(argv);
                return null;
            }
            case "Main.solveProblem":
                return statNoTime((long[]) call(null, MAIN, "solveProblem", new Class<?>[]{PRB, BO, DB},
                        P(c, a, "p"), B(a, "solveLP"), D(a, "timeLimit")), 1);
            case "Main.solveProblemPMG":
                return statNoTime((long[]) call(null, MAIN, "solveProblemPMG", new Class<?>[]{PRB, BO, DB},
                        P(c, a, "p"), B(a, "solveLP"), D(a, "timeLimit")), 1);
            case "Main.solveProblemSuboptimal":
                return statNoTime((long[]) call(null, MAIN, "solveProblemSuboptimal",
                        new Class<?>[]{PRB, BO, BO, IN, DB, IN, BO}, P(c, a, "p"), B(a, "solveLP"), B(a, "setOptimal"),
                        I(a, "extraSteps"), D(a, "timeLimit"), I(a, "splitLevel"), B(a, "noCycle")), 1);
            case "Main.solveProblemTotalTime":
                return statNoTime((long[]) call(null, MAIN, "solveProblemTotalTime", new Class<?>[]{PRB, DB, DB},
                        P(c, a, "p"), D(a, "gap"), D(a, "timeLimit")), 1);
            case "Main.solveProblemDistance":
                return statNoTime((long[]) call(null, MAIN, "solveProblemDistance", new Class<?>[]{PRB, DB, DB},
                        P(c, a, "p"), D(a, "gap"), D(a, "timeLimit")), 1);
            case "Main.solveProblemDistanceSplit":
                return statNoTime((long[]) call(null, MAIN, "solveProblemDistanceSplit", new Class<?>[]{PRB, DB, DB, IN},
                        P(c, a, "p"), D(a, "gap"), D(a, "timeLimit"), I(a, "splits")), 1);
            case "Main.solvePuzzleProblem":
                return statNoTime((long[]) call(null, MAIN, "solvePuzzleProblem", new Class<?>[]{IN, PRB, DB},
                        I(a, "instance"), P(c, a, "p"), D(a, "timeLimit")), 1);
            case "Main.solveProblemWithHeuristic":
                return statNoTime((long[]) call(null, MAIN, "solveProblemWithHeuristic", new Class<?>[]{PRB},
                        P(c, a, "p")), 1);
            case "Main.statsSequence": {
                // reset*Stats(test); process*Stats(stat) for each injected stat vector; write*Stats().
                boolean heur = "heuristic".equals(a.get("kind"));
                call(null, MAIN, heur ? "resetHeuristicStats" : "resetStats", new Class<?>[]{String.class}, S(a, "test"));
                for (Object o : (List<Object>) req(a, "stats")) {
                    long[] st = null;
                    if (o != null) {
                        List<Object> l = (List<Object>) o;
                        st = new long[l.size()];
                        for (int i = 0; i < st.length; i++) st[i] = ((Number) l.get(i)).longValue();
                    }
                    if (heur) call(null, MAIN, "processHeuristicStats", new Class<?>[]{long[].class, IN}, st, I(a, "n"));
                    else call(null, MAIN, "processStats", new Class<?>[]{long[].class}, (Object) st);
                }
                if (!Boolean.FALSE.equals(a.get("write")))
                    call(null, MAIN, heur ? "writeHeuristicStats" : "writeStats", new Class<?>[]{});
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("currentTest", Main.currentTest);
                m.put("subTotal", Main.subTotal);
                m.put("pathLengths", Main.pathLengths);
                m.put("totalTime", Main.totalTime);
                m.put("successCount", Main.successCount);
                m.put("goalsReached", Main.goalsReached);
                m.put("pathLength", Main.pathLength);
                m.put("expPathLength", Main.expPathLength);
                m.put("finishingTimeSet", toJson(Main.finishingTimeSet));
                m.put("makeSpanTotal", Main.makeSpanTotal);
                return m;
            }
            case "Main.writePuzzleStats":
                call(null, MAIN, "writePuzzleStats", new Class<?>[]{IN, DB}, I(a, "instance"), D(a, "time"));
                return null;
            case "Main.writeString":
                call(null, MAIN, "writeString", new Class<?>[]{String.class}, S(a, "str"));
                return null;
            case "Main.writePuzzleResult": {
                Graph g = G(c, a, "g");
                call(null, MAIN, "writePuzzleResult", new Class<?>[]{IN, DB, GR, IA2C}, I(a, "instance"), D(a, "time"),
                        g, IA2(c, a, "paths"));
                return null;
            }
            default:
                throw new IllegalArgumentException("unknown fn " + fn);
        }
    }

    // ------------------------------------------------------------------------------------
    // Minimal JSON (objects -> LinkedHashMap, arrays -> ArrayList, integers -> Long,
    // other numbers -> Double)
    // ------------------------------------------------------------------------------------

    static final class Json {
        private final String s;
        private int i = 0;

        private Json(String s) {
            this.s = s;
        }

        static Object parse(String s) {
            Json j = new Json(s);
            j.ws();
            Object v = j.value();
            j.ws();
            if (j.i != s.length()) throw new IllegalArgumentException("trailing data at " + j.i);
            return v;
        }

        private void ws() {
            while (i < s.length() && Character.isWhitespace(s.charAt(i))) i++;
        }

        private Object value() {
            char ch = s.charAt(i);
            if (ch == '{') {
                i++;
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                ws();
                if (s.charAt(i) == '}') {
                    i++;
                    return m;
                }
                while (true) {
                    ws();
                    String k = string();
                    ws();
                    expect(':');
                    ws();
                    m.put(k, value());
                    ws();
                    if (s.charAt(i) == ',') {
                        i++;
                        continue;
                    }
                    expect('}');
                    return m;
                }
            }
            if (ch == '[') {
                i++;
                List<Object> l = new ArrayList<Object>();
                ws();
                if (s.charAt(i) == ']') {
                    i++;
                    return l;
                }
                while (true) {
                    ws();
                    l.add(value());
                    ws();
                    if (s.charAt(i) == ',') {
                        i++;
                        continue;
                    }
                    expect(']');
                    return l;
                }
            }
            if (ch == '"') return string();
            if (s.startsWith("true", i)) {
                i += 4;
                return Boolean.TRUE;
            }
            if (s.startsWith("false", i)) {
                i += 5;
                return Boolean.FALSE;
            }
            if (s.startsWith("null", i)) {
                i += 4;
                return null;
            }
            if (s.startsWith("NaN", i)) {
                i += 3;
                return Double.NaN;
            }
            if (s.startsWith("Infinity", i)) {
                i += 8;
                return Double.POSITIVE_INFINITY;
            }
            if (s.startsWith("-Infinity", i)) {
                i += 9;
                return Double.NEGATIVE_INFINITY;
            }
            int st = i;
            while (i < s.length() && "+-0123456789.eE".indexOf(s.charAt(i)) >= 0) i++;
            String t = s.substring(st, i);
            if (t.isEmpty()) throw new IllegalArgumentException("bad JSON at " + st);
            if (t.indexOf('.') < 0 && t.indexOf('e') < 0 && t.indexOf('E') < 0) return Long.parseLong(t);
            return Double.parseDouble(t);
        }

        private void expect(char ch) {
            if (s.charAt(i) != ch) throw new IllegalArgumentException("expected '" + ch + "' at " + i);
            i++;
        }

        private String string() {
            expect('"');
            StringBuilder sb = new StringBuilder();
            while (true) {
                char ch = s.charAt(i++);
                if (ch == '"') return sb.toString();
                if (ch != '\\') {
                    sb.append(ch);
                    continue;
                }
                char e = s.charAt(i++);
                switch (e) {
                    case 'n': sb.append('\n'); break;
                    case 't': sb.append('\t'); break;
                    case 'r': sb.append('\r'); break;
                    case 'b': sb.append('\b'); break;
                    case 'f': sb.append('\f'); break;
                    case 'u':
                        sb.append((char) Integer.parseInt(s.substring(i, i + 4), 16));
                        i += 4;
                        break;
                    default: sb.append(e);
                }
            }
        }

        static String emit(Object o) {
            StringBuilder sb = new StringBuilder();
            emit(o, sb);
            return sb.toString();
        }

        @SuppressWarnings("unchecked")
        static void emit(Object o, StringBuilder sb) {
            if (o == null) sb.append("null");
            else if (o instanceof String) str((String) o, sb);
            else if (o instanceof Boolean) sb.append(o.toString());
            else if (o instanceof Double || o instanceof Float) {
                double d = ((Number) o).doubleValue();
                if (Double.isNaN(d)) sb.append("\"NaN\"");
                else if (Double.isInfinite(d)) sb.append(d > 0 ? "\"Infinity\"" : "\"-Infinity\"");
                else sb.append(Double.toString(d));
            } else if (o instanceof Number) sb.append(((Number) o).longValue());
            else if (o instanceof Map) {
                sb.append('{');
                boolean first = true;
                for (Map.Entry<String, Object> e : ((Map<String, Object>) o).entrySet()) {
                    if (!first) sb.append(',');
                    first = false;
                    str(e.getKey(), sb);
                    sb.append(':');
                    emit(e.getValue(), sb);
                }
                sb.append('}');
            } else if (o instanceof List) {
                sb.append('[');
                boolean first = true;
                for (Object x : (List<Object>) o) {
                    if (!first) sb.append(',');
                    first = false;
                    emit(x, sb);
                }
                sb.append(']');
            } else str(o.toString(), sb);
        }

        static void str(String s, StringBuilder sb) {
            sb.append('"');
            for (int k = 0; k < s.length(); k++) {
                char ch = s.charAt(k);
                switch (ch) {
                    case '"': sb.append("\\\""); break;
                    case '\\': sb.append("\\\\"); break;
                    case '\n': sb.append("\\n"); break;
                    case '\r': sb.append("\\r"); break;
                    case '\t': sb.append("\\t"); break;
                    default:
                        if (ch < 0x20 || ch > 0x7e) sb.append(String.format("\\u%04x", (int) ch));
                        else sb.append(ch);
                }
            }
            sb.append('"');
        }
    }
}
