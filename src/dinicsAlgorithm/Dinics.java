package dinicsAlgorithm;

/**
 * Implementation of Dinic's network flow algorithm. The algorithm works by first constructing a
 * level graph using a BFS and then finding augmenting paths on the level graph using multiple DFSs.
 *
 * <p>Run script:
 *
 * <p>$ ./gradlew run -Palgorithm=graphtheory.networkflow.Dinics
 *
 * <p>Time Complexity: O(EV²)
 *
 * @author William Fiset, william.alexandre.fiset@gmail.com
 */

import static java.lang.Math.min;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;

public class Dinics {
	public static class Edge {
		public int from, to;
		public Edge residual;
		public long flow, cost;
		public final long capacity, originalCost;

		public Edge(int from, int to, long capacity) {
			this(from, to, capacity, 0 /* unused */);
		}

		public Edge(int from, int to, long capacity, long cost) {
			this.from = from;
			this.to = to;
			this.capacity = capacity;
			this.originalCost = this.cost = cost;
		}

		public boolean isResidual() {
			return capacity == 0;
		}

		public long remainingCapacity() {
			return capacity - flow;
		}

		public void augment(long bottleNeck) {
			flow += bottleNeck;
			residual.flow -= bottleNeck;
		}

		public String toString(int s, int t) {
			String u = (from == s) ? "s" : ((from == t) ? "t" : String.valueOf(from));
			String v = (to == s) ? "s" : ((to == t) ? "t" : String.valueOf(to));
			return String.format("Edge %s -> %s | flow = %d | capacity = %d | is residual: %s", u, v, flow, capacity,
					isResidual());
		}
	}

	public abstract static class NetworkFlowSolverBase {

		// To avoid overflow, set infinity to a value less than Long.MAX_VALUE;
		protected static final long INF = Long.MAX_VALUE / 2;

		// Inputs: n = number of nodes, s = source, t = sink
		protected final int n, s, t;

		protected long maxFlow;
		protected long minCost;

		protected boolean[] minCut;
		protected List<Edge>[] graph;

		// 'visited' and 'visitedToken' are variables used for graph sub-routines to
		// track whether a node has been visited or not. In particular, node 'i' was
		// recently visited if visited[i] == visitedToken is true. This is handy
		// because to mark all nodes as unvisited simply increment the visitedToken.
		private int visitedToken = 1;
		private int[] visited;

		// Indicates whether the network flow algorithm has ran. We should not need to
		// run the solver multiple times, because it always yields the same result.
		private boolean solved;

		/**
		 * Creates an instance of a flow network solver. Use the {@link #addEdge} method
		 * to add edges to the graph.
		 *
		 * @param n - The number of nodes in the graph including source and sink nodes.
		 * @param s - The index of the source node, 0 <= s < n
		 * @param t - The index of the sink node, 0 <= t < n, t != s
		 */
		public NetworkFlowSolverBase(int n, int s, int t) {
			this.n = n;
			this.s = s;
			this.t = t;
			initializeGraph();
			minCut = new boolean[n];
			visited = new int[n];
		}

		// Construct an empty graph with n nodes including the source and sink nodes.
		@SuppressWarnings("unchecked")
		private void initializeGraph() {
			graph = new List[n];
			for (int i = 0; i < n; i++)
				graph[i] = new ArrayList<Edge>();
		}

		/**
		 * Adds a directed edge (and residual edge) to the flow graph.
		 *
		 * @param from     - The index of the node the directed edge starts at.
		 * @param to       - The index of the node the directed edge ends at.
		 * @param capacity - The capacity of the edge.
		 */
		public void addEdge(int from, int to, long capacity) {
			if (capacity < 0)
				throw new IllegalArgumentException("Capacity < 0");
			Edge e1 = new Edge(from, to, capacity);
			Edge e2 = new Edge(to, from, 0);
			e1.residual = e2;
			e2.residual = e1;
			graph[from].add(e1);
			graph[to].add(e2);
		}

		/** Cost variant of {@link #addEdge(int, int, int)} for min-cost max-flow */
		public void addEdge(int from, int to, long capacity, long cost) {
			Edge e1 = new Edge(from, to, capacity, cost);
			Edge e2 = new Edge(to, from, 0, -cost);
			e1.residual = e2;
			e2.residual = e1;
			graph[from].add(e1);
			graph[to].add(e2);
		}

		// Marks node 'i' as visited.
		public void visit(int i) {
			visited[i] = visitedToken;
		}

		// Returns whether or not node 'i' has been visited.
		public boolean visited(int i) {
			return visited[i] == visitedToken;
		}

		// Resets all nodes as unvisited. This is especially useful to do
		// between iterations of finding augmenting paths, O(1)
		public void markAllNodesAsUnvisited() {
			visitedToken++;
		}

		/**
		 * Returns the graph after the solver has been executed. This allow you to
		 * inspect the {@link Edge#flow} compared to the {@link Edge#capacity} in each
		 * edge. This is useful if you want to figure out which edges were used during
		 * the max flow.
		 */
		public List<Edge>[] getGraph() {
			execute();
			return graph;
		}

		// Returns the maximum flow from the source to the sink.
		public long getMaxFlow() {
			execute();
			return maxFlow;
		}

		// Returns the min cost from the source to the sink.
		// NOTE: This method only applies to min-cost max-flow algorithms.
		public long getMinCost() {
			execute();
			return minCost;
		}

		// Returns the min-cut of this flow network in which the nodes on the "left
		// side"
		// of the cut with the source are marked as true and those on the "right side"
		// of the cut with the sink are marked as false.
		public boolean[] getMinCut() {
			execute();
			return minCut;
		}

		// Wrapper method that ensures we only call solve() once
		private void execute() {
			if (solved)
				return;
			solved = true;
			solve();
		}

		// Method to implement which solves the network flow problem.
		public abstract void solve();
	}

	public static class DinicsSolver extends NetworkFlowSolverBase {

		private int[] level;

		/**
		 * Creates an instance of a flow network solver. Use the {@link #addEdge} method
		 * to add edges to the graph.
		 *
		 * @param n - The number of nodes in the graph including source and sink nodes.
		 * @param s - The index of the source node, 0 <= s < n
		 * @param t - The index of the sink node, 0 <= t < n, t != s
		 */
		public DinicsSolver(int n, int s, int t) {
			super(n, s, t);
			level = new int[n];
		}

		@Override
		public void solve() {
			// next[i] indicates the next unused edge index in the adjacency list for node
			// i. This is part
			// of the Shimon Even and Alon Itai optimization of pruning deads ends as part
			// of the DFS phase.
			int[] next = new int[n];

			while (bfs()) {
				Arrays.fill(next, 0);
				// Find max flow by adding all augmenting path flows.
				for (long f = dfs(s, next, INF); f != 0; f = dfs(s, next, INF)) {
					maxFlow += f;
					System.out.println("flow: " + f);
				}
			}

			for (int i = 0; i < n; i++)
				if (level[i] != -1)
					minCut[i] = true;
		}

		// Do a BFS from source to sink and compute the depth/level of each node
		// which is the minimum number of edges from that node to the source.
		private boolean bfs() {
			Arrays.fill(level, -1);
			level[s] = 0;
			Deque<Integer> q = new ArrayDeque<>(n);
			q.offer(s);
			while (!q.isEmpty()) {
				int node = q.poll();
				for (Edge edge : graph[node]) {
					long cap = edge.remainingCapacity();
					if (cap > 0 && level[edge.to] == -1) {
						level[edge.to] = level[node] + 1;
						q.offer(edge.to);
					}
				}
			}
			return level[t] != -1;
		}

		private long dfs(int at, int[] next, long flow) {
			if (at == t)
				return flow;
			final int numEdges = graph[at].size();

			for (; next[at] < numEdges; next[at]++) {
				Edge edge = graph[at].get(next[at]);
				long cap = edge.remainingCapacity();
				if (cap > 0 && level[edge.to] == level[at] + 1) {

					long bottleNeck = dfs(edge.to, next, min(flow, cap));
					if (bottleNeck > 0) {
						edge.augment(bottleNeck);
						return bottleNeck;
					}
				}
			}
			return 0;
		}

	}
	/* Examples */

	public static void main(String[] args) {
		testSmallFlowGraph();
		System.out.println("===============================================");
		testGraphFromSlides();
	}

// Testing graph from:
// http://crypto.cs.mcgill.ca/~crepeau/COMP251/KeyNoteSlides/07demo-maxflowCS-C.pdf
	private static void testSmallFlowGraph() {
		int n = 6;
		int s = n - 1;
		int t = n - 2;

		DinicsSolver solver;
		solver = new DinicsSolver(n, s, t);

		// Source edges
		solver.addEdge(s, 0, 10);
		solver.addEdge(s, 1, 10);

		// Sink edges
		solver.addEdge(2, t, 10);
		solver.addEdge(3, t, 10);

		// Middle edges
		solver.addEdge(0, 1, 2);
		solver.addEdge(0, 2, 4);
		solver.addEdge(0, 3, 8);
		solver.addEdge(1, 3, 9);
		solver.addEdge(3, 2, 6);

		System.out.printf("Dinics Maximum flow: %d\n", solver.getMaxFlow()); // 19
		List<Edge>[] resultGraph = solver.getGraph();

		// Displays all edges part of the resulting residual graph.
		for (List<Edge> edges : resultGraph)
			for (Edge e : edges)
				System.out.println(e.toString(s, t));
	}

	private static void testGraphFromSlides() {
		int n = 11;
		int s = n - 1;
		int t = n - 2;

		DinicsSolver solver;
		solver = new DinicsSolver(n, s, t);

		// Source edges
		solver.addEdge(s, 0, 5);
		solver.addEdge(s, 1, 10);
		solver.addEdge(s, 2, 15);

		// Middle edges
		solver.addEdge(0, 3, 10);
		solver.addEdge(1, 0, 15);
		solver.addEdge(1, 4, 20);
		solver.addEdge(2, 5, 25);
		solver.addEdge(3, 4, 25);
		solver.addEdge(3, 6, 10);
		solver.addEdge(3, 7, 20);
		solver.addEdge(4, 2, 5);
		solver.addEdge(4, 7, 30);
		solver.addEdge(5, 7, 20);
		solver.addEdge(5, 8, 10);
		solver.addEdge(7, 8, 15);

		// Sink edges
		solver.addEdge(6, t, 5);
		solver.addEdge(7, t, 15);
		solver.addEdge(8, t, 10);

		System.out.printf("Dinics Maximum flow: %d\n", solver.getMaxFlow()); // 30
		List<Edge>[] resultGraph = solver.getGraph();

		// Displays all edges part of the resulting residual graph.
		for (List<Edge> edges : resultGraph)
			for (Edge e : edges)
				System.out.println(e.toString(s, t));
	}
}