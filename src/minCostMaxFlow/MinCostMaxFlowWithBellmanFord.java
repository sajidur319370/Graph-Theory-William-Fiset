package minCostMaxFlow;

/**
 * Min Cost Max Flow algorithm implemented with Bellman-Ford as a means of finding augmenting paths
 * to support negative edge weights.
 *
 * <p>Time Complexity: O(E²V²)
 *
 * @author William Fiset, william.alexandre.fiset@gmail.com
 */

import static java.lang.Math.min;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class MinCostMaxFlowWithBellmanFord {
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

	public static class BellmanFordSolver extends NetworkFlowSolverBase {

		/**
		 * Creates a min-cost maximum flow network solver. To construct the flow network
		 * use the {@link NetworkFlowSolverBase#addEdge} method to add edges to the
		 * graph.
		 *
		 * @param n - The number of nodes in the graph including source and sink nodes.
		 * @param s - The index of the source node, 0 <= s < n
		 * @param t - The index of the sink node, 0 <= t < n, t != s
		 */
		public BellmanFordSolver(int n, int s, int t) {
			super(n, s, t);
		}

		@Override
		public void solve() {

			// Sum up the bottlenecks on each augmenting path to find the max flow and min
			// cost.
			List<Edge> path;
			while ((path = getAugmentingPath()).size() != 0) {

				// Find bottle neck edge value along path.
				long bottleNeck = Long.MAX_VALUE;
				for (Edge edge : path)
					bottleNeck = min(bottleNeck, edge.remainingCapacity());

				// Retrace path while augmenting the flow
				for (Edge edge : path) {
					edge.augment(bottleNeck);
					minCost += bottleNeck * edge.originalCost;
				}
				maxFlow += bottleNeck;
			}

			// TODO(williamfiset): Compute mincut.
		}

		/**
		 * Use the Bellman-Ford algorithm (which work with negative edge weights) to
		 * find an augmenting path through the flow network.
		 */
		private List<Edge> getAugmentingPath() {
			long[] dist = new long[n];
			Arrays.fill(dist, INF);
			dist[s] = 0;

			Edge[] prev = new Edge[n];

			// For each vertex, relax all the edges in the graph, O(VE)
			for (int i = 0; i < n - 1; i++) {
				for (int from = 0; from < n; from++) {
					for (Edge edge : graph[from]) {
						if (edge.remainingCapacity() > 0 && dist[from] + edge.cost < dist[edge.to]) {
							dist[edge.to] = dist[from] + edge.cost;
							prev[edge.to] = edge;
						}
					}
				}
			}

			// Retrace augmenting path from sink back to the source.
			LinkedList<Edge> path = new LinkedList<>();
			for (Edge edge = prev[t]; edge != null; edge = prev[edge.from])
				path.addFirst(edge);
			return path;
		}

	}
	/* Example usage. */

	public static void main(String[] args) {
		testSmallNetwork();
	}

	private static void testSmallNetwork() {
		int n = 6;
		int s = n - 1;
		int t = n - 2;
		BellmanFordSolver solver;
		solver = new BellmanFordSolver(n, s, t);

		solver.addEdge(s, 1, 4, 10);
		solver.addEdge(s, 2, 2, 30);
		solver.addEdge(1, 2, 2, 10);
		solver.addEdge(1, t, 0, 9999);
		solver.addEdge(2, t, 4, 10);

		// Prints: Max flow: 4, Min cost: 140
		System.out.printf("Bellman Ford Max flow: %d, Min cost: %d\n", solver.getMaxFlow(), solver.getMinCost());
	}
}