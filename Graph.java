package AlgoritimodeDijkastra;

import java.util.*;

public class Graph {
    private int vertices;
    private LinkedList<Edge>[] adjacencyList;

    class Edge {
        int target, weight;
        Edge(int target, int weight) {
            this.target = target;
            this.weight = weight;
        }
    }

    Graph(int vertices) {
        this.vertices = vertices;
        adjacencyList = new LinkedList[vertices];
        for (int i = 0; i < vertices; i++) {
            adjacencyList[i] = new LinkedList<>();
        }
    }

    void addEdge(int source, int target, int weight) {
        adjacencyList[source].add(new Edge(target, weight));
        adjacencyList[target].add(new Edge(source, weight)); // For undirected graph
    }

    void dijkstra(int source) {
        int[] dist = new int[vertices];
        boolean[] visited = new boolean[vertices];
        PriorityQueue<Edge> pq = new PriorityQueue<>(vertices, Comparator.comparingInt(edge -> edge.weight));

        Arrays.fill(dist, Integer.MAX_VALUE);
        dist[source] = 0;
        pq.add(new Edge(source, 0));

        while (!pq.isEmpty()) {
            int u = pq.poll().target;
            if (visited[u]) continue;
            visited[u] = true;

            for (Edge edge : adjacencyList[u]) {
                int v = edge.target;
                int weight = edge.weight;

                if (!visited[v] && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.add(new Edge(v, dist[v]));
                }
            }
        }

        printSolution(dist);
    }

    void printSolution(int[] dist) {
        System.out.println("Vertex \t\t Distance from Source");
        for (int i = 0; i < vertices; i++) {
            System.out.println(i + " \t\t " + dist[i]);
        }
    }

    // Placeholder methods for minimum spanning tree and maximum flow
    void minimumSpanningTree() {
        // Implement Kruskal's or Prim's algorithm here
boolean[] mstSet = new boolean[vertices];
int[] key = new int[vertices];
int[] parent = new int[vertices];
PriorityQueue<Edge> pq = new PriorityQueue<>(vertices, Comparator.comparingInt(edge -> edge.weight));

Arrays.fill(key, Integer.MAX_VALUE);
key[0] = 0;
parent[0] = -1;
pq.add(new Edge(0, key[0]));

while (!pq.isEmpty()) {
    int u = pq.poll().target;
    mstSet[u] = true;

    for (Edge edge : adjacencyList[u]) {
    int v = edge.target;
    int weight = edge.weight;

    if (!mstSet[v] && weight < key[v]) {
        key[v] = weight;
        pq.add(new Edge(v, key[v]));
        parent[v] = u;
    }
    }
}

printMST(parent);
    }

    void printMST(int[] parent) {
System.out.println("Edge \tWeight");
for (int i = 1; i < vertices; i++) {
    for (Edge edge : adjacencyList[i]) {
    if (edge.target == parent[i]) {
        System.out.println(parent[i] + " - " + i + "\t" + edge.weight);
        break;
    }
    }
}
    }

    // void maximumFlow() {
    //     // Implement Ford-Fulkerson algorithm here
    // }

    class FlowEdge {
        int source, target, capacity, flow;

        FlowEdge(int source, int target, int capacity) {
            this.source = source;
            this.target = target;
            this.capacity = capacity;
            this.flow = 0;
        }
    }

    void maximumFlow(int source, int sink) {
        int u, v;
        int[][] residualGraph = new int[vertices][vertices];

        for (u = 0; u < vertices; u++) {
            for (Edge edge : adjacencyList[u]) {
                residualGraph[u][edge.target] = edge.weight;
            }
        }

        int[] parent = new int[vertices];
        int maxFlow = 0;

        while (bfs(residualGraph, source, sink, parent)) {
            int pathFlow = Integer.MAX_VALUE;
            for (v = sink; v != source; v = parent[v]) {
                u = parent[v];
                pathFlow = Math.min(pathFlow, residualGraph[u][v]);
            }

            for (v = sink; v != source; v = parent[v]) {
                u = parent[v];
                residualGraph[u][v] -= pathFlow;
                residualGraph[v][u] += pathFlow;
            }

            maxFlow += pathFlow;
        }

        System.out.println("The maximum possible flow is " + maxFlow);
    }

    boolean bfs(int[][] residualGraph, int source, int sink, int[] parent) {
        boolean[] visited = new boolean[vertices];
        Queue<Integer> queue = new LinkedList<>();
        queue.add(source);
        visited[source] = true;
        parent[source] = -1;

        while (!queue.isEmpty()) {
            int u = queue.poll();

            for (int v = 0; v < vertices; v++) {
                if (!visited[v] && residualGraph[u][v] > 0) {
                    queue.add(v);
                    parent[v] = u;
                    visited[v] = true;
                }
            }
        }

        return visited[sink];
    }

    public static void main(String[] args) {
        Graph graph = new Graph(5);
        graph.addEdge(0, 1, 9);
        graph.addEdge(0, 2, 6);
        graph.addEdge(0, 3, 5);
        graph.addEdge(0, 4, 3);
        graph.addEdge(2, 1, 2);
        graph.addEdge(2, 3, 4);

        graph.dijkstra(0);
        // graph.minimumSpanningTree();
        // graph.maximumFlow();
    long startTime, endTime;
    Runtime runtime = Runtime.getRuntime();
    long memoryBefore, memoryAfter;

    // Measure Dijkstra's algorithm
    runtime.gc();
    memoryBefore = runtime.totalMemory() - runtime.freeMemory();
    startTime = System.nanoTime();
    graph.dijkstra(0);
    endTime = System.nanoTime();
    memoryAfter = runtime.totalMemory() - runtime.freeMemory();
    System.out.println("Dijkstra's algorithm:");
    System.out.println("Execution time: " + (endTime - startTime) + " ns");
    System.out.println("Memory usage: " + (memoryAfter - memoryBefore) + " bytes");

    // Measure Minimum Spanning Tree
    runtime.gc();
    memoryBefore = runtime.totalMemory() - runtime.freeMemory();
    startTime = System.nanoTime();
    graph.minimumSpanningTree();
    endTime = System.nanoTime();
    memoryAfter = runtime.totalMemory() - runtime.freeMemory();
    System.out.println("Minimum Spanning Tree:");
    System.out.println("Execution time: " + (endTime - startTime) + " ns");
    System.out.println("Memory usage: " + (memoryAfter - memoryBefore) + " bytes");

    // Measure Maximum Flow
    runtime.gc();
    memoryBefore = runtime.totalMemory() - runtime.freeMemory();
    startTime = System.nanoTime();
    graph.maximumFlow(0, 4);
    endTime = System.nanoTime();
    memoryAfter = runtime.totalMemory() - runtime.freeMemory();
    System.out.println("Maximum Flow:");
    System.out.println("Execution time: " + (endTime - startTime) + " ns");
    System.out.println("Memory usage: " + (memoryAfter - memoryBefore) + " bytes");
    }
}