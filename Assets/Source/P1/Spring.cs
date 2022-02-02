using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring {

    #region InEditorVariables

    public float Stiffness;
    public float Damping;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public enum SpringType { Stretch, Bend };
    public SpringType springType;

    public float Length0;
    public float Length;
    public Vector3 dir;

    private PhysicsManager Manager;

    public Spring(Node a, Node b, SpringType s)
    {
        nodeA = a;
        nodeB = b;
        springType = s;
    }

    // Use this for initialization
    public void Initialize(float stiffness, float damping, PhysicsManager m)
    {
        Stiffness = stiffness;
        Damping = damping;
        m = Manager;
        
        UpdateState();
        Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        dir = nodeA.Pos - nodeB.Pos;
        Length = dir.magnitude;
        dir = (1.0f / Length) * dir;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        // Add Hooke's law and damping forces related with actual nodes vel
        // Direction of the Forces
        Vector3 u = nodeA.Pos - nodeB.Pos;
        u.Normalize();
        
        // Elastic Force
        Vector3 Force = - Stiffness * (Length - Length0) * u;
        // Damping Force
        Force += - Damping * Vector3.Dot(u, nodeA.Vel - nodeB.Vel) * u;
        
        // Node A
        force[nodeA.index] += Force.x;
        force[nodeA.index + 1] += Force.y;
        force[nodeA.index + 2] += Force.z;
        
        // Node B
        force[nodeB.index] -= Force.x;
        force[nodeB.index + 1] -= Force.y;
        force[nodeB.index + 2] -= Force.z;
    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // Direction of the Forces
        VectorXD u = new DenseVectorXD(3);
        Vector3 dir = nodeA.Pos - nodeB.Pos;
        dir.Normalize();
        u[0] = dir[0];
        u[1] = dir[1];
        u[2] = dir[2];
        MatrixXD uuT = u.OuterProduct(u);
        
        // Identity matrix
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);
        
        // dFadxa and cross derivatives computation
        MatrixXD dFadxa = - Stiffness * (Length - Length0) / Length * I - Stiffness * (Length0 / Length) * uuT;
        MatrixXD dFadxb = - dFadxa;
        MatrixXD dFbdxa = - dFadxa;
        MatrixXD dFbdxb = dFadxa;
        
        // dFadva and cross derivatives computation dFadva => -d * u.transposed * u
        MatrixXD dFadva = - Damping * uuT;
        MatrixXD dFadvb = - dFadva;
        MatrixXD dFbdva = - dFadva;
        MatrixXD dFbdvb = dFadva;
        
        // Fill dFdx (K) matrix
        // dFadxa
        dFdx.SetSubMatrix(nodeA.index, 
                        nodeA.index, 
                          dFdx.SubMatrix(nodeA.index, 3, nodeA.index, 3) + dFadxa);
        
        // dFadxb
        dFdx.SetSubMatrix(nodeA.index, 
                        nodeB.index, 
                          dFdx.SubMatrix(nodeA.index, 3, nodeB.index, 3) + dFadxb);

        // dFbdxa
        dFdx.SetSubMatrix(nodeB.index, 
                        nodeA.index, 
                          dFdx.SubMatrix(nodeB.index, 3, nodeA.index, 3) + dFbdxa);

        // dFbdxb
        dFdx.SetSubMatrix(nodeB.index, 
                        nodeB.index, 
                          dFdx.SubMatrix(nodeB.index, 3, nodeB.index, 3) + dFbdxb);
        
        // Fill dFdv (D) matrix
        // dFadva
        dFdv.SetSubMatrix(nodeA.index, 
                        nodeA.index, 
                          dFdv.SubMatrix(nodeA.index, 3, nodeA.index, 3) + dFadva);
        
        // dFadvb
        dFdv.SetSubMatrix(nodeA.index, 
                        nodeB.index, 
                          dFdv.SubMatrix(nodeA.index, 3, nodeB.index, 3) + dFadvb);

        // dFbdva
        dFdv.SetSubMatrix(nodeB.index, 
                        nodeA.index, 
                          dFdv.SubMatrix(nodeB.index, 3, nodeA.index, 3) + dFbdva);

        // dFbdvb
        dFdv.SetSubMatrix(nodeB.index, 
                        nodeB.index, 
                          dFdv.SubMatrix(nodeB.index, 3, nodeB.index, 3) + dFbdvb);
    }

}
