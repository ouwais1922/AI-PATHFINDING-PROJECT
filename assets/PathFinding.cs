using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PathFinding : MonoBehaviour {

	public Transform seeker, target;
	Grid grid;

	void Awake() {
		grid = GetComponent<Grid> ();
	}

	void Update() {
		var watchAstar = new System.Diagnostics.Stopwatch(); 
		var watchAstarManhattan = new System.Diagnostics.Stopwatch();
		var watchAstarEuclidian = new System.Diagnostics.Stopwatch();
		var watchUCS = new System.Diagnostics.Stopwatch();
		var watchBFS = new System.Diagnostics.Stopwatch();
		var watchDFS = new System.Diagnostics.Stopwatch();

		watchAstar.Start();
		FindPathAStar (seeker.position, target.position);
		watchAstar.Stop();
		

		watchAstarManhattan.Start();
		FindPathStarManhattan (seeker.position, target.position);
		watchAstarManhattan.Stop();

		watchAstarEuclidian.Start();
		FindPathAStarEuclidean (seeker.position, target.position);
		watchAstarEuclidian.Stop();

		watchUCS.Start();
		FindPathUCS (seeker.position, target.position);
		watchUCS.Stop();

		watchBFS.Start();
		FindPathBFS (seeker.position, target.position);
		watchBFS.Stop();

		watchDFS.Start();
		FindPathDFS (seeker.position, target.position);
		watchDFS.Stop();

		Debug.Log($"Execution Time A*: {watchAstar.ElapsedMilliseconds}ms \n Execution Time Astar manhattan : {watchAstarManhattan.ElapsedMilliseconds}ms \n Execution Time Astar Euclidian : {watchAstarEuclidian.ElapsedMilliseconds}ms\n Execution Time Astar BFS : {watchBFS.ElapsedMilliseconds}ms\n Execution Time Astar UCS : {watchUCS.ElapsedMilliseconds}ms\n Execution Time Astar dfs : {watchDFS.ElapsedMilliseconds}ms");

	}

	void FindPathAStar(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				List<Node> path = RetracePath(startNode,targetNode, "astar");
				string info = "FindPathAStar: \nFringe";
				info += printExpandedNodesLists(openSet);
				info += "\nFringe Nodes Count: ";
				info += openSet.Count;
				info += "\nExpanded Nodes: ";
				info += printInfo(closedSet);
				info += "\nExpanded Nodes Count: ";
				info += closedSet.Count;
				info += "\nPath Length: ";
				info += path.Count;
				Debug.Log(info);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void FindPathAStarEuclidean(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				List<Node> path = RetracePath(startNode,targetNode, "astareuclidean");
				string info = "FindPathAStarEuclidean: \n Fringe";
				info += printExpandedNodesLists(openSet);
				info += "\nFringe Nodes Count: ";
				info += openSet.Count;
				info += "\nExpanded Nodes: ";
				info += printInfo(closedSet);
				info += "\nExpanded Nodes Count: ";
				info += closedSet.Count;
				info += "\nPath Length: ";
				info += path.Count;
				Debug.Log(info);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetEuclideanDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void FindPathStarManhattan(Vector3 startPos,Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode) {
				List<Node> path = RetracePath(startNode,targetNode, "astarmanhattan");
				string info = "FindPathStarManhattan: \n Fringe";
				info += printExpandedNodesLists(openSet);
				info += "\nFringe Nodes Count: ";
				info += openSet.Count;
				info += "\nExpanded Nodes: ";
				info += printInfo(closedSet);
				info += "\nExpanded Nodes Count: ";
				info += closedSet.Count;
				info += "\nPath Length: ";
				info += path.Count;
				Debug.Log(info);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetManhattanDistance(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}

			
	} 

	void FindPathBFS(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
        Queue<Node> openSet = new Queue<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Enqueue(startNode);
		closedSet.Add(startNode);
        while (openSet.Count > 0)
        {
            var node = openSet.Dequeue();
            if (node == targetNode) {
				List<Node> path = RetracePath(startNode,targetNode, "bfs");
				string info = "FindPathbfs: \n Fringe";
				info += printExpandedNodesQueues(openSet);
				info += "\nFringe Nodes Count: ";
				info += openSet.Count;
				info += "\nExpanded Nodes: ";
				info += printInfo(closedSet);
				info += "\nExpanded Nodes Count: ";
				info += closedSet.Count;
				info += "\nPath Length: ";
				info += path.Count;
				Debug.Log(info);
				return;
			}
            foreach (Node neighbour in grid.GetNeighbours(node)) {
                if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
                if (!closedSet.Contains(neighbour))
					neighbour.parent = node;
					openSet.Enqueue(neighbour);
 					closedSet.Add(neighbour);
            }
        }
	}



	void FindPathDFS(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
        Stack<Node> openSet = new Stack<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Push(startNode);
        while (openSet.Count > 0)
        {
            var node = openSet.Pop();
            if (closedSet.Contains(node))
                continue;
            else
                closedSet.Add(node);
            if (node == targetNode) {
				List<Node> path = RetracePath(startNode,targetNode, "dfs");
				string info = "FindPathDFS: \n Fringe";
				info += printExpandedNodesStacks(openSet);
				info += "\nFringe Nodes Count: ";
				info += openSet.Count;
				info += "\nExpanded Nodes: ";
				info += printInfo(closedSet);
				info += "\nExpanded Nodes Count: ";
				info += closedSet.Count;
				info += "\nPath Length: ";
				info += path.Count;
				Debug.Log(info);
				return;
			}
            foreach (Node neighbour in grid.GetNeighbours(node)) {
                if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}
				closedSet.Add(node);
                neighbour.parent = node;
                if (!openSet.Contains(neighbour))
					openSet.Push(neighbour);
            }
        }
	}

	void FindPathUCS(Vector3 startPos, Vector3 targetPos) {
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		startNode.gCost = 0;

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0) {
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i ++) {
				if (openSet[i].gCost <= node.gCost) {
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			if (closedSet.Contains(node))
                continue;
            else
                closedSet.Add(node);

			if (node == targetNode) {
				List<Node> path = RetracePath(startNode,targetNode, "ucs");
				string info = "FindPathUCS: \n Fringe";
				info += printExpandedNodesLists(openSet);
				info += "\nFringe Nodes Count: ";
				info += openSet.Count;
				info += "\nExpanded Nodes: ";
				info += printInfo(closedSet);
				info += "\nExpanded Nodes Count: ";
				info += closedSet.Count;
				info += "\nPath Length: ";
				info += path.Count;
				Debug.Log(info);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)) {
				if (!neighbour.walkable || closedSet.Contains(neighbour)) {
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
					neighbour.gCost = newCostToNeighbour;
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
		Debug.Log("FindPathUCS:");
		printInfo(closedSet);
	}

	List<Node> RetracePath(Node startNode, Node endNode, string algo) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		path.Reverse();
		if(algo == "astar")
			grid.pathAStar = path;
		if(algo == "bfs")
			grid.pathBFS = path;
		if(algo == "dfs")
			grid.pathDFS = path;
		if(algo == "ucs")
			grid.pathUCS = path;
		if(algo == "astareuclidean")
			grid.pathAStarEuclidean = path;
		if(algo == "astarmanhattan")
			grid.pathAStarManhattan = path;
		return path;
	}

	string printInfo(HashSet<Node> closedSet){
		string fringe = "";
		int count = 0;
		foreach(var ele in closedSet)
        {
			count++;
            fringe += "(" + ele.gridX + ";" + ele.gridY + ")" + ", ";
			if(count >=100)
				break;
        }

		return fringe;
	}
	string printExpandedNodesStacks( Stack<Node>  expandedNodes){
		string enodes = "";
		foreach(var ele in expandedNodes)
        {
            enodes += "(" + ele.gridX + ";" + ele.gridY + ")" + ", ";
        }
		return enodes;
	}
	string printExpandedNodesQueues(  Queue<Node> expandedNodes){
		string enodes = "";
		foreach(var ele in expandedNodes)
        {
            enodes += "(" + ele.gridX + ";" + ele.gridY + ")" + ", ";
        }
		return enodes;
	}
	string printExpandedNodesLists(	List<Node> expandedNodes){
		string enodes = "";
		foreach(var ele in expandedNodes)
        {
            enodes += "(" + ele.gridX + ";" + ele.gridY + ")" + ", ";
        }
		return enodes;
	}

	int GetDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);
	}

	int GetEuclideanDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		return (int)Math.Floor(Math.Sqrt(dstX*dstX + dstY*dstY) * 10);
	}
	int GetManhattanDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		return dstX * 10 + dstY *10;
	}
}
