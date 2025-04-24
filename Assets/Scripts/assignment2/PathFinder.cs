using UnityEngine;
using System.Collections.Generic;
using System.Linq; // Required for OrderBy

// Helper class for the Priority Queue
public class PriorityQueueNode
{
    public GraphNode Node { get; }
    public float Priority { get; } // fScore

    public PriorityQueueNode(GraphNode node, float priority)
    {
        Node = node;
        Priority = priority;
    }
}

public class PathFinder : MonoBehaviour
{
    // Assignment 2: Implement AStar
    //
    // DO NOT CHANGE THIS SIGNATURE (parameter types + return type)
    // AStar will be given the start node, destination node and the target position, and should return
    // a path as a list of positions the agent has to traverse to reach its destination, as well as the
    // number of nodes that were expanded to find this path
    // The last entry of the path will be the target position, and you can also use it to calculate the heuristic
    // value of nodes you add to your search frontier; the number of expanded nodes tells us if your search was
    // efficient
    //
    // Take a look at StandaloneTests.cs for some test cases
    public static (List<Vector3>, int) AStar(GraphNode start, GraphNode destination, Vector3 target)
    {
        int expandedNodes = 0; // Counter for expanded nodes

        // Heuristic function: Euclidean distance from node center to target position
        float Heuristic(GraphNode node)
        {
            return Vector3.Distance(node.GetCenter(), target);
        }

        // Stores nodes to visit, ordered by fScore.
         var openSet = new SortedList<float, PriorityQueueNode>(); 
         var openSetTracker = new HashSet<GraphNode>();

        var cameFrom = new Dictionary<GraphNode, GraphNode>();

        var gScore = new Dictionary<GraphNode, float>();

        // how short a path from start to finish can be if it goes through n.
        var fScore = new Dictionary<GraphNode, float>();

        // Initialize scores for the start node
        gScore[start] = 0;
        fScore[start] = Heuristic(start);

        // Add start node to the open set
         openSet.Add(fScore[start], new PriorityQueueNode(start, fScore[start]));
         openSetTracker.Add(start);


        while (openSet.Count > 0)
        {
            // Get the node in openSet having the lowest fScore value
            PriorityQueueNode currentWrapper = openSet.Values[0];
            GraphNode current = currentWrapper.Node;
            openSet.RemoveAt(0); // Remove the node with the lowest fScore
            openSetTracker.Remove(current);


            expandedNodes++; // Increment expanded node count

            // Check if we reached the destination
            if (current == destination)
            {
                // Reconstruct path
                List<Vector3> path = new List<Vector3>();
                GraphNode temp = current;
                while (cameFrom.ContainsKey(temp))
                {
                    // Add the center of the node to the path
                    path.Add(temp.GetCenter());
                    temp = cameFrom[temp];
                }
                // Add the start node's center
                path.Add(start.GetCenter());
                path.Reverse(); // Reverse to get path from start to destination

                // add the exact target click position as the final point
                 path.Add(target);

                return (path, expandedNodes);
            }

            // Explore neighbors
            foreach (var neighborEdge in current.GetNeighbors()) //
            {
                GraphNode neighbor = neighborEdge.GetNode(); //

                // tentative_gScore is the distance from start to the neighbor through current
                // distance between node centers as edge cost
                float tentative_gScore = gScore[current] + Vector3.Distance(current.GetCenter(), neighbor.GetCenter());


                // Initialize gScore for neighbor if not seen before
                if (!gScore.ContainsKey(neighbor))
                {
                    gScore[neighbor] = float.PositiveInfinity;
                }

                 // This path to neighbor is better than any previous one. Record it!
                if (tentative_gScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentative_gScore;
                    fScore[neighbor] = gScore[neighbor] + Heuristic(neighbor);

                    // If neighbor is not in openSet, add it.
                    // if nodes are revisited with better scores frequently.
                     if (!openSetTracker.Contains(neighbor))
                     {
                         float key = fScore[neighbor];
                         while (openSet.ContainsKey(key)) key += float.Epsilon;

                         openSet.Add(key, new PriorityQueueNode(neighbor, fScore[neighbor]));
                         openSetTracker.Add(neighbor);
                     }
                }
            }
        }

        // Open set is empty but goal was never reached
        return (new List<Vector3>(), expandedNodes); // Return empty path and expanded count
    }

    public Graph graph;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        EventBus.OnTarget += PathFind;
        EventBus.OnSetGraph += SetGraph;
    }

    // Update is called once per frame
    void Update()
    {

    }

    public void SetGraph(Graph g)
    {
        graph = g;
    }

    // entry point
    public void PathFind(Vector3 target)
    {
        if (graph == null) return;

        // find start and destination nodes in graph
        GraphNode start = null;
        GraphNode destination = null;
        foreach (var n in graph.all_nodes) //
        {
            if (Util.PointInPolygon(transform.position, n.GetPolygon())) //
            {
                start = n;
            }
            if (Util.PointInPolygon(target, n.GetPolygon())) //
            {
                destination = n;
            }
        }

        if (start != null && destination != null)
        {
            // only find path if destination is inside graph
            EventBus.ShowTarget(target);
            (List<Vector3> path, int expanded) = PathFinder.AStar(start, destination, target);

            Debug.Log("found path of length " + path.Count + " expanded " + expanded + " nodes, out of: " + (graph.all_nodes?.Count ?? 0)); //
            EventBus.SetPath(path);
        }
        else
        {
            // Handle cases where start or destination is outside the navigable graph area
             Debug.LogWarning("Start or Destination node not found in the graph. Cannot calculate path.");
             EventBus.SetPath(null); // Clear any existing path visualization
        }
    }
}