import java.util.*;

/**
 * Delivery Route Optimizer  (DSA-3, 25CS2103E)
 * Graph (adjacency list) + Min-Heap priority queue + Dijkstra's algorithm.
 *
 * Compile: javac RouteOptimizer.java
 * Run    : java RouteOptimizer
 */
public class RouteOptimizer {

    // ---------- Edge of the graph ----------
    static class Edge {
        int to, weight;
        Edge(int to, int weight) { this.to = to; this.weight = weight; }
    }

    // ---------- Min-Heap priority queue (built from scratch) ----------
    static class MinHeap {
        private int[][] heap = new int[16][];   // each entry: {distance, node}
        private int size = 0;

        boolean isEmpty() { return size == 0; }

        void push(int dist, int node) {
            if (size == heap.length) heap = Arrays.copyOf(heap, size * 2);
            heap[size] = new int[]{dist, node};
            int i = size++;
            while (i > 0) {                       // sift up
                int parent = (i - 1) / 2;
                if (heap[parent][0] <= heap[i][0]) break;
                swap(i, parent);
                i = parent;
            }
        }

        int[] pop() {
            int[] top = heap[0];
            heap[0] = heap[--size];
            int i = 0;
            while (true) {                        // sift down
                int l = 2 * i + 1, r = l + 1, min = i;
                if (l < size && heap[l][0] < heap[min][0]) min = l;
                if (r < size && heap[r][0] < heap[min][0]) min = r;
                if (min == i) break;
                swap(i, min);
                i = min;
            }
            return top;
        }

        private void swap(int a, int b) { int[] t = heap[a]; heap[a] = heap[b]; heap[b] = t; }
    }

    // ---------- Graph using adjacency lists ----------
    static class Graph {
        final int n;
        final String[] names;
        final List<List<Edge>> adj = new ArrayList<>();

        Graph(String[] names) {
            this.n = names.length;
            this.names = names;
            for (int i = 0; i < n; i++) adj.add(new ArrayList<>());
        }

        void addRoad(int a, int b, int km) {      // undirected road
            adj.get(a).add(new Edge(b, km));
            adj.get(b).add(new Edge(a, km));
        }

        int indexOf(String name) {
            for (int i = 0; i < n; i++)
                if (names[i].equalsIgnoreCase(name)) return i;
            return -1;
        }

        // Dijkstra from src. Fills dist[] and prev[].
        void dijkstra(int src, int[] dist, int[] prev) {
            Arrays.fill(dist, Integer.MAX_VALUE);
            Arrays.fill(prev, -1);
            dist[src] = 0;
            MinHeap pq = new MinHeap();
            pq.push(0, src);
            while (!pq.isEmpty()) {
                int[] top = pq.pop();
                int d = top[0], u = top[1];
                if (d > dist[u]) continue;        // stale entry
                for (Edge e : adj.get(u)) {
                    if (d + e.weight < dist[e.to]) {
                        dist[e.to] = d + e.weight;
                        prev[e.to] = u;
                        pq.push(dist[e.to], e.to);
                    }
                }
            }
        }

        List<Integer> path(int[] prev, int target) {
            LinkedList<Integer> p = new LinkedList<>();
            for (int v = target; v != -1; v = prev[v]) p.addFirst(v);
            return p;
        }

        String pathString(List<Integer> p) {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < p.size(); i++) {
                if (i > 0) sb.append(" -> ");
                sb.append(names[p.get(i)]);
            }
            return sb.toString();
        }

        // Total distance visiting stops in the given order (depot = 0)
        int routeDistance(List<Integer> order, boolean returnToDepot) {
            int cur = 0, total = 0;
            int[] dist = new int[n], prev = new int[n];
            List<Integer> targets = new ArrayList<>(order);
            if (returnToDepot) targets.add(0);
            for (int t : targets) {
                dijkstra(cur, dist, prev);
                if (dist[t] == Integer.MAX_VALUE) return -1;   // unreachable
                total += dist[t];
                cur = t;
            }
            return total;
        }

        // Greedy: always go to the nearest unvisited stop (by shortest-path distance)
        List<Integer> optimizedOrder(List<Integer> stops) {
            List<Integer> remaining = new ArrayList<>(stops), order = new ArrayList<>();
            int[] dist = new int[n], prev = new int[n];
            int cur = 0;
            while (!remaining.isEmpty()) {
                dijkstra(cur, dist, prev);
                int best = remaining.get(0);
                for (int s : remaining) if (dist[s] < dist[best]) best = s;
                order.add(best);
                remaining.remove(Integer.valueOf(best));
                cur = best;
            }
            return order;
        }
    }

    // ---------- Output helpers ----------
    static String seq(Graph g, List<Integer> order, boolean back) {
        StringBuilder sb = new StringBuilder(g.names[0]);
        for (int i : order) sb.append(" -> ").append(g.names[i]);
        if (back) sb.append(" -> ").append(g.names[0]);
        return sb.toString();
    }

    static void solve(Graph g, List<Integer> stops, boolean back) {
        int[] dist = new int[g.n], prev = new int[g.n];
        g.dijkstra(0, dist, prev);

        System.out.println("\nShortest distances from depot (" + g.names[0] + "):");
        System.out.printf("%-14s%-12s%s%n", "Delivery Point", "Distance", "Path Used");
        for (int s : stops) {
            if (dist[s] == Integer.MAX_VALUE) {
                System.out.printf("%-14s%-12s%s%n", g.names[s], "unreachable", "-");
            } else {
                System.out.printf("%-14s%-12s%s%n", g.names[s], dist[s] + " km",
                        g.pathString(g.path(prev, s)));
            }
        }

        List<Integer> opt = g.optimizedOrder(stops);
        int optKm = g.routeDistance(opt, back);
        int naiveKm = g.routeDistance(stops, back);
        if (optKm < 0 || naiveKm < 0) {
            System.out.println("\nSome stops are not connected to the depot.");
            return;
        }
        System.out.println("\nOptimized sequence: " + seq(g, opt, back) + "  (" + optKm + " km)");
        System.out.println("Naive sequence    : " + seq(g, stops, back) + "  (" + naiveKm + " km)");
        int saved = naiveKm - optKm;
        int pct = naiveKm == 0 ? 0 : Math.round(100f * saved / naiveKm);
        System.out.println("Distance saved    : " + saved + " km (" + pct + "%)");
    }

    // ---------- Sample data (same as the website demo) ----------
    static void runDemo() {
        Graph g = new Graph(new String[]{"Depot", "A", "B", "C", "D", "E"});
        g.addRoad(0, 1, 4);  g.addRoad(0, 3, 7);  g.addRoad(1, 2, 5);  g.addRoad(3, 4, 5);
        g.addRoad(1, 3, 6);  g.addRoad(2, 3, 8);  g.addRoad(2, 5, 6);  g.addRoad(4, 5, 4);
        solve(g, Arrays.asList(1, 2, 3, 4, 5), false);
    }

    // ---------- Simple console interface ----------
    static void runCustom(Scanner sc) {
        System.out.print("Number of locations (including depot): ");
        int n = Integer.parseInt(sc.nextLine().trim());
        String[] names = new String[n];
        names[0] = "Depot";
        System.out.println("Location 1 is the Depot. Enter names for the other locations:");
        for (int i = 1; i < n; i++) {
            System.out.print("  Location " + (i + 1) + " name: ");
            names[i] = sc.nextLine().trim();
        }
        Graph g = new Graph(names);

        System.out.print("Number of roads: ");
        int m = Integer.parseInt(sc.nextLine().trim());
        System.out.println("Enter each road as:  <from> <to> <km>   (e.g. Depot A 4)");
        for (int i = 0; i < m; i++) {
            String[] p = sc.nextLine().trim().split("\\s+");
            int a = g.indexOf(p[0]), b = g.indexOf(p[1]);
            if (a < 0 || b < 0) { System.out.println("  Unknown location, try again."); i--; continue; }
            g.addRoad(a, b, Integer.parseInt(p[2]));
        }

        System.out.print("Delivery stops (space separated names): ");
        List<Integer> stops = new ArrayList<>();
        for (String s : sc.nextLine().trim().split("\\s+")) {
            int idx = g.indexOf(s);
            if (idx > 0 && !stops.contains(idx)) stops.add(idx);
        }
        System.out.print("Return to depot at the end? (y/n): ");
        boolean back = sc.nextLine().trim().equalsIgnoreCase("y");
        solve(g, stops, back);
    }

    public static void main(String[] args) {
        Scanner sc = new Scanner(System.in);
        System.out.println("=== Delivery Route Optimizer (Dijkstra + Min-Heap) ===");
        System.out.println("1. Run sample demo");
        System.out.println("2. Enter my own network");
        System.out.print("Choose: ");
        String choice = sc.nextLine().trim();
        if (choice.equals("2")) runCustom(sc); else runDemo();
    }
}
