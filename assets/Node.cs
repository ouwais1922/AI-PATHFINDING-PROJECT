using UnityEngine;
using System.Collections;

public class Node {
	
	public bool walkable;
	public Vector3 worldPosition;
	public int gridX;
	public int gridY;

	public int gCost;
	public int hCost;
	public Node parent;
	
	public Node(bool _walkable, Vector3 _worldPos, int _gridX, int _gridY) {
		walkable = _walkable;
		worldPosition = _worldPos;
		gridX = _gridX;
		gridY = _gridY;
	}

	public int fCost {
		get {
			return gCost + hCost;
		}
	}
	public override bool Equals(object obj)
    {
        return obj is Node && Equals((Node) obj);
    }

    public bool Equals(Node n)
    {
        return gridX == n.gridX && gridY == n.gridY;
    }
}