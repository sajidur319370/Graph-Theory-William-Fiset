package tarjanStronglyConnectedComponents;

/**
 * An implementation of Tarjan's Strongly Connected Components algorithm using an adjacency list.
 *
 * <p>Verified against:
 *
 * <ul>
 *   <li>https://open.kattis.com/problems/equivalences
 *   <li>https://open.kattis.com/problems/runningmom
 *   <li>https://www.hackerearth.com/practice/algorithms/graphs/strongly-connected-components/tutorial
 * </ul>
 *
 * <p>Time complexity: O(V+E)
 *
 * @author William Fiset, william.alexandre.fiset@gmail.com
 */

import static java.lang.Math.min;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TarjanSccSolverAdjacencyList {

	private int n;
	private List<List<Integer>> graph;

	private boolean solved;
	private int sccCount, id;
	private boolean[] visited;
	private int[] ids, low, sccs;
	private Deque<Integer> stack;

	private static final int UNVISITED = -1;

	public TarjanSccSolverAdjacencyList(List<List<Integer>> graph) {
		if (graph == null)
			throw new IllegalArgumentException("Graph cannot be null.");
		n = graph.size();
		this.graph = graph;
	}

	// Returns the number of strongly connected components in the graph.
	public int sccCount() {
		if (!solved)
			solve();
		return sccCount;
	}

	// Get the connected components of this graph. If two indexes
	// have the same value then they're in the same SCC.
	public int[] getSccs() {
		if (!solved)
			solve();
		return sccs;
	}

	public void solve() {
		if (solved)
			return;

		ids = new int[n];
		low = new int[n];
		sccs = new int[n];
		visited = new boolean[n];
		stack = new ArrayDeque<>();
		Arrays.fill(ids, UNVISITED);

		for (int i = 0; i < n; i++) {
			System.out.println("i:" + i);
			if (ids[i] == UNVISITED) {
				System.out.println("i:" + i);
				System.out.println("id:" + id);
				System.out.println("SCCCount:" + sccCount);
				dfs(i);

			}
		}

		solved = true;
	}

	private void dfs(int at) {

		ids[at] = low[at] = id++;
		System.out.println("ids1:" + ids[at]);
		System.out.println("low1:" + low[at]);
		System.out.println("Id " + id);
		stack.push(at);
		System.out.println("Stack first:" + stack.getFirst());
		visited[at] = true;
		System.out.printf("visited[%d]: %b \n", at, visited[at]);

		for (int to : graph.get(at)) {
			if (ids[to] == UNVISITED) {
				dfs(to);
			}
			if (visited[to]) {
				low[at] = min(low[at], low[to]);
			}
			System.out.println("at:  " + at);
		}
		System.out.println("ids:" + ids[at]);
		System.out.println("low:" + low[at]);

		// On recursive callback, if we're at the root node (start of SCC)
		// empty the seen stack until back to root.
		if (ids[at] == low[at]) {
			for (int node = stack.pop();; node = stack.pop()) {
				visited[node] = false;
				System.out.println("node:" + node);
				sccs[node] = sccCount;
				System.out.println("Scc: " + "[" + node + "]" + sccCount);
				if (node == at) {
					System.out.println("Before break:");
					break;
				}
				System.out.println("After break:");

			}
			sccCount++;
			System.out.println("sccCount:" + sccCount);
		}
	}

	// Initializes adjacency list with n nodes.
	public static List<List<Integer>> createGraph(int n) {
		List<List<Integer>> graph = new ArrayList<>(n);
		for (int i = 0; i < n; i++)
			graph.add(new ArrayList<>());
		return graph;
	}

	// Adds a directed edge from node 'from' to node 'to'
	public static void addEdge(List<List<Integer>> graph, int from, int to) {
		graph.get(from).add(to);
	}

	/* Example usage: */

	public static void main(String[] arg) {
		int n = 5;
		List<List<Integer>> graph = createGraph(n);

		addEdge(graph, 0, 3);
		addEdge(graph, 3, 1);
		addEdge(graph, 3, 4);
		addEdge(graph, 4, 0);
		addEdge(graph, 1, 2);
		addEdge(graph, 4, 2);
		addEdge(graph, 2, 1);

		TarjanSccSolverAdjacencyList solver = new TarjanSccSolverAdjacencyList(graph);

		int[] sccs = solver.getSccs();
		Map<Integer, List<Integer>> multimap = new HashMap<>();
		for (int i = 0; i < n; i++) {
			if (!multimap.containsKey(sccs[i]))
				multimap.put(sccs[i], new ArrayList<>());
			multimap.get(sccs[i]).add(i);
		}

		// Prints:
		// Number of Strongly Connected Components: 2
		// Nodes: [1, 2] form a Strongly Connected Component.
		// Nodes: [0, 3, 4] form a Strongly Connected Component.
		System.out.printf("Number of Strongly Connected Components: %d\n", solver.sccCount());
		for (List<Integer> scc : multimap.values()) {
			System.out.println("Nodes: " + scc + " form a Strongly Connected Component.");
		}
	}
}