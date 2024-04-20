import java.util.*;

public class AStar {
    static class Node {
        int id;
        List<Edge> neighbors;

        public Node(int id) {
            this.id = id;
            this.neighbors = new ArrayList<>();
        }

        public void addNeighbor(Edge edge) {
            neighbors.add(edge);
        }

        @Override
        public String toString() {
            return "Node-" + id;
        }
    }

    static class Edge {
        Node target;
        int weight;

        public Edge(Node target, int weight) {
            this.target = target;
            this.weight = weight;
        }
    }

    static class PathNode implements Comparable<PathNode> {
        Node node;
        int gScore;
        int fScore;

        public PathNode(Node node, int gScore, int fScore) {
            this.node = node;
            this.gScore = gScore;
            this.fScore = fScore;
        }

        @Override
        public int compareTo(PathNode other) {
            return Integer.compare(this.fScore, other.fScore);
        }
    }

    public static List<Node> astar(Node[] graph, Node source, Node destination) {
        Map<Node, Integer> gScores = new HashMap<>();
        Map<Node, Integer> fScores = new HashMap<>();
        Map<Node, Node> cameFrom = new HashMap<>();

        for (Node node : graph) {
            gScores.put(node, Integer.MAX_VALUE);
            fScores.put(node, Integer.MAX_VALUE);
        }

        gScores.put(source, 0);
        fScores.put(source, heuristic(source, destination));

        PriorityQueue<PathNode> openSet = new PriorityQueue<>();
        openSet.offer(new PathNode(source, 0, heuristic(source, destination)));

        while (!openSet.isEmpty()) {
            PathNode current = openSet.poll();
            Node current_node = current.node;

            if (current_node == destination) {
                return reconstructPath(cameFrom, destination);
            }

            for (Edge neighborEdge : current_node.neighbors) {
                Node neighbor = neighborEdge.target;
                int tentative_gScore = gScores.get(current_node) + neighborEdge.weight;

                if (tentative_gScore < gScores.get(neighbor)) {
                    cameFrom.put(neighbor, current_node);
                    gScores.put(neighbor, tentative_gScore);
                    fScores.put(neighbor, tentative_gScore + heuristic(neighbor, destination));
                    openSet.offer(new PathNode(neighbor, tentative_gScore, fScores.get(neighbor)));
                }
            }
        }

        return null; // No path found
    }

    static int heuristic(Node a, Node b) {
        // Implement heuristic function here (e.g., Euclidean distance, Manhattan distance, etc.)
        return 0; // Placeholder heuristic value
    }

    static List<Node> reconstructPath(Map<Node, Node> cameFrom, Node current) {
        List<Node> path = new ArrayList<>();
        while (cameFrom.containsKey(current)) {
            path.add(0, current);
            current = cameFrom.get(current);
        }
        path.add(0, current);
        return path;
    }

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);
        System.out.print("Enter the number of nodes: ");
        int numNodes = scanner.nextInt();
        Node[] graph = new Node[numNodes];

        System.out.print("Enter the number of edges: ");
        int numEdges = scanner.nextInt();

        for (int i = 0; i < numNodes; i++) {
            graph[i] = new Node(i);
        }

        System.out.println("Enter edge details (source destination weight):");
        for (int i = 0; i < numEdges; i++) {
            int sourceId = scanner.nextInt();
            int targetId = scanner.nextInt();
            int weight = scanner.nextInt();
            graph[sourceId].addNeighbor(new Edge(graph[targetId], weight));
        }

        System.out.print("Enter the source node id: ");
        int sourceId = scanner.nextInt();
        Node source = graph[sourceId];

        System.out.print("Enter the destination node id: ");
        int destinationId = scanner.nextInt();
        Node destination = graph[destinationId];

        List<Node> shortestPath = astar(graph, source, destination);
        if (shortestPath != null) {
            System.out.println("Shortest path: " + shortestPath);
        } else {
            System.out.println("No path found.");
        }
        scanner.close();
    }
    /*
     * Test Case:
     * Input:
     * Enter the number of nodes: 4
     * Enter the number of edges: 5
     * Enter edge details (source destination weight):
     * 0 1 4
     * 0 2 2
     * 1 2 1
     * 1 3 5
     * 2 3 8
     * Enter the source node id: 0
     * Enter the destination node id: 3
     * 
     * Output:
     * Shortest path: [Node-0, Node-1, Node-3]
     */
}

