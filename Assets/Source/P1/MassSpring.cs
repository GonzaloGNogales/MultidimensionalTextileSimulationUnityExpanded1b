using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic mass-spring model component which can be dropped onto
/// a game object and configured so that the set of nodes and
/// edges behave as a mass-spring model.
/// </summary>
public class MassSpring : MonoBehaviour, ISimulable
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public MassSpring()
    {
	    Manager = null;
    }

    #region EditorVariables

    public List<Node> Nodes;
    public List<Spring> Springs;

    public float Mass;
    public float StiffnessStretch;
    public float StiffnessBend;
    public float DampingAlpha;
    public float DampingBeta;

    #endregion

    #region OtherVariables
    private PhysicsManager Manager;
    private int index;
    private class Edge {
        private int vertexA;
        private int vertexB;
        private int vertexOther;
	    
        public Edge(int vA, int vB, int vO) {
            if (vA <= vB) {  // The smallest vertex index is always in the first position
                vertexA = vA;
                vertexB = vB;
            }
            else {
                vertexA = vB;
                vertexB = vA;
            }

            vertexOther = vO;
        }

        public int GetVertexA() {
            return vertexA;
        }
	    
        public int GetVertexB() {
            return vertexB;
        }

        public int GetVertexOther() {
            return vertexOther;
        }
	    
        public override bool Equals(object obj) {
            // If the passed object is null
            if (obj == null)
                return false;
            if (!(obj is Edge))
                return false;
            return (vertexA == ((Edge)obj).vertexA)
                   && (vertexB == ((Edge)obj).vertexB);
        }
	    
        public override int GetHashCode() {
            return (vertexA, vertexB, vertexOther).GetHashCode();
        }
    }
    #endregion

    #region MonoBehaviour

    public void Awake()
    {
	    // Initialize Nodes and Springs lists
	    Nodes = new List<Node>();
	    Springs = new List<Spring>();
	    
	    // Retrieve the mesh filter component
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        
        // Assign each vertex of the mesh a node behaviour
        Debug.Log("Assigning each vertex of the mesh a node behaviour...");
        Vector3[] vertices = mesh.vertices;  // Retrieve mesh vertices

	    foreach (Vector3 vertex in vertices)
	    {
		    Vector3 vertexPos = transform.TransformPoint(vertex);  // Locate the nodes in the positions of the mesh vertices (Global positions)
		    Node node = new Node(vertexPos);  // Create a Node object with global position
		    Nodes.Add(node);  // Finally add node to the nodes list to iterate through all of them on fixed update loop
	    }

	    // Assign each edge of the mesh a spring behaviour
	    Debug.Log("Assigning each edge of the mesh a spring behaviour...");
	    int[] triangles = mesh.triangles;
	    List<Edge> edges = new List<Edge>();
	    
	    // Create all NodeA - NodeB springs and assign them an stiffness k
	    if (triangles.Length % 3 == 0) {  // Check if the number of indices is exact for a number of triangles
		    int topologyIdx = 1;
		    for (int i = 0; i < triangles.Length / 3; ++i) {  // Initialize edges vertex _triangles indices
			    // POSSIBLE OPTIMIZATION FOR MILLION VERTEX MESHES: CREATE NODES IN THIS LOOP!!! (Triangle iteration)
			    // Edges list creation
			    edges.Add(new Edge(triangles[topologyIdx - 1], triangles[topologyIdx], triangles[topologyIdx + 1]));
			    edges.Add(new Edge(triangles[topologyIdx], triangles[topologyIdx + 1], triangles[topologyIdx - 1]));
			    edges.Add(new Edge(triangles[topologyIdx + 1], triangles[topologyIdx - 1], triangles[topologyIdx]));
			    topologyIdx += 3;  // Update the _triangle access index (TopologyIdx) to step for the next 3 edges iter.
		    }

		    // Sort edges list by vertexA and vertexB values (that represent the indices of the triangles)
		    edges.Sort((e1, e2) =>
		    {
			    int vAcomparison = e1.GetVertexA().CompareTo(e2.GetVertexA());
			    if (vAcomparison == 0) return e1.GetVertexB().CompareTo(e2.GetVertexB());
			    return vAcomparison;
		    });
		    
		    // Iterate edges list for creating springs (stretching and bending)
		    for (int i = 0; i < edges.Count; ++i) {
			    if (i != edges.Count - 1 && edges[i].Equals(edges[i + 1])) {  // Duplicated edge is found
				    // ADD STRETCHING SPRING FROM VERTEX_A TO VERTEX_B
				    // Create the actual stretching spring and save it
				    Spring stretchingSpring = new Spring(Nodes[edges[i].GetVertexA()], Nodes[edges[i].GetVertexB()], Spring.SpringType.Stretch);
				    Springs.Add(stretchingSpring);
				    
				    // ADD BENDING SPRING FROM VERTEX_A TO VERTEX_B
				    // Create the actual bending spring and save it
				    Spring bendingSpring = new Spring(Nodes[edges[i].GetVertexOther()], Nodes[edges[i + 1].GetVertexOther()], Spring.SpringType.Bend);
				    Springs.Add(bendingSpring);

				    // Skip the bending spring index on the edges list and continue iterating
				    i++;
			    }
			    else {  // Actual edge is not duplicated
				    // IN THIS CASE ALWAYS ADD STRETCHING SPRING FROM VERTEX_A TO VERTEX_B
				    // Create the actual stretching spring and save it
				    Spring stretchingSpring = new Spring(Nodes[edges[i].GetVertexA()], Nodes[edges[i].GetVertexB()], Spring.SpringType.Stretch);
				    Springs.Add(stretchingSpring);
			    }
		    }
	    }
	    
	    Debug.Log("Nodes and Springs initialization finished successfully!!!!");
    }

    public void Update()
    {
	    // Retrieve the mesh filter component
	    Mesh mesh = GetComponent<MeshFilter>().mesh;
	    Vector3[] vertices = mesh.vertices;  // Retrieve mesh vertices
	    // Iterate through every vertex of the mesh and assign them their new position value, previously computed in nodes list
	    for (int i = 0; i < Nodes.Count; i++) vertices[i] = transform.InverseTransformPoint(Nodes[i].Pos);
	    mesh.vertices = vertices;  // Update mesh vertices position (Local positions)
	    // For the springs it is enough to perform this node assignation in the FixedUpdate as long as
	    // springs are only used to compute the elastic forces and apply them to the nodes, so only updating nodes will work
    }

    public void FixedUpdate()
    {
        // TO BE COMPLETED
    }
    #endregion

    #region ISimulable

    public void Initialize(int ind, PhysicsManager m, List<Fixer> fixers)
    {
        Manager = m;			 // Physics Manager object reference
        index = ind;			 // Starting node index => 0
        Mass = 1;                // Each node mass value
        StiffnessStretch = 100;  // Stretching stiffness (Traction Springs)
        StiffnessBend = 10;      // Bending stiffness << Stretching stiffness
        DampingAlpha = 0.4f;     // Nodes: alpha * mass
        DampingBeta = 0.01f;     // Springs: beta * stiffness

        // Initialize the nodes with required values
        foreach (Node node in Nodes)
        {
	        node.Initialize(index, Mass, DampingAlpha, Manager);
	        index += 3;  // Update node index value for the next node, 3 DoFs per node, so += 3
        }
        
        // Initialize the stretching and bending springs with required values
        foreach (Spring spring in Springs)
        {
	        if (spring.springType == Spring.SpringType.Stretch) spring.Initialize(StiffnessStretch, DampingBeta, Manager);
	        else spring.Initialize(StiffnessBend, DampingBeta, Manager);
        }
        
        // Finally iterate through every fixer in the scene and check if any of the mesh nodes is inside its bounds
        foreach (Fixer fixer in fixers)
        {
            foreach (Node node in Nodes)
            {
                if (fixer.IsInside(node.Pos))
                {
                    node.Fixed = true;
                }
            }
        }
    }

    public int GetNumDoFs()
    {
        return 3 * Nodes.Count;
    }

    public void GetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetPosition(position);
    }

    public void SetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetPosition(position);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].UpdateState();
    }

    public void GetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetVelocity(velocity);
    }

    public void SetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetVelocity(velocity);
    }

    public void GetForce(VectorXD force)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForce(force);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForce(force);
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForceJacobian(dFdx, dFdv);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForceJacobian(dFdx, dFdv);
    }

    public void GetMass(MatrixXD mass)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMass(mass);
    }

    public void GetMassInverse(MatrixXD massInv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMassInverse(massInv);
    }

    public void FixVector(VectorXD v)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixVector(v);
        }
    }

    public void FixMatrix(MatrixXD M)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixMatrix(M);
        }
    }

    #endregion

    #region OtherMethods

    #endregion

}
