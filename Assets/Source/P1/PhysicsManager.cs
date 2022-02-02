using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		Paused = true;
		TimeStep = 0.01f;
		Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		IntegrationMethod = Integration.Explicit;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		Symplectic = 1,
        Implicit = 2,
	};

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
    public List<GameObject> SimObjects;
    public List<Fixer> Fixers;
    public Integration IntegrationMethod;
    public int Substeps;

    #endregion

    #region OtherVariables
    private List<ISimulable> m_objs;
    private int m_numDoFs;
    #endregion

    #region MonoBehaviour

    public void Start()
    {
	    //Parse the simulable objects and initialize their state indices
        m_numDoFs = 0;
        m_objs = new List<ISimulable>(SimObjects.Capacity);

        foreach (GameObject obj in SimObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();
            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable object
                simobj.Initialize(m_numDoFs, this, Fixers);

                // Retrieve pos and vel size
                m_numDoFs += simobj.GetNumDoFs();
            }
        }
    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			Paused = !Paused;
	}

    public void FixedUpdate()
    {
        if (Paused)
            return; // Not simulating

        // Select integration method and substepping
        for (int i = 0; i < Substeps; ++i)
        {
	        switch (IntegrationMethod)
	        {
		        case Integration.Explicit: stepExplicit(); break;
		        case Integration.Symplectic: stepSymplectic(); break;
		        case Integration.Implicit: stepImplicit(); break;
		        default:
			        throw new System.Exception("[ERROR] Should never happen!");
	        }
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step using Explicit integration.
    /// </summary>
    private void stepExplicit()
	{
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x += TimeStep * v;
        v += TimeStep * (Minv * f);

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Symplectic integration.
    /// </summary>
    private void stepSymplectic()
	{
		VectorXD x = new DenseVectorXD(m_numDoFs);
		VectorXD v = new DenseVectorXD(m_numDoFs);
		VectorXD f = new DenseVectorXD(m_numDoFs);
		f.Clear();
		MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
		Minv.Clear();

		foreach (ISimulable obj in m_objs)
		{
			obj.GetPosition(x);
			obj.GetVelocity(v);
			obj.GetForce(f);
			obj.GetMassInverse(Minv);
		}

		foreach (ISimulable obj in m_objs)
		{
			obj.FixVector(f);
			obj.FixMatrix(Minv);
		}
		
		v += TimeStep * (Minv * f);
		x += TimeStep * v;
		
		foreach (ISimulable obj in m_objs)
		{
			obj.SetPosition(x);
			obj.SetVelocity(v);
		}
    }

    /// <summary>
    /// Performs a simulation step using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        VectorXD b = new DenseVectorXD(m_numDoFs);

        MatrixXD A = new DenseMatrixXD(m_numDoFs);
        MatrixXD M = new DenseMatrixXD(m_numDoFs);
        MatrixXD dFdx = new DenseMatrixXD(m_numDoFs);  // Elastic force derivative
        MatrixXD dFdv = new DenseMatrixXD(m_numDoFs);  // Damping force derivative
        
        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMass(M);
            obj.GetForceJacobian(dFdx, dFdv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(M);
            obj.FixMatrix(dFdx);
            obj.FixMatrix(dFdv);
        }

        // The velocity implicit integration is computed solving a linear system
        A = M - TimeStep * dFdv - TimeStep * TimeStep * dFdx;
        b = (M - TimeStep * dFdv) * v + TimeStep * f;
        v = A.Solve(b);  // Solving A * v(t+h) = b => v(t+h)
        // The position integration is the same as it was already implicit using Symplectic Integration Method
        x += TimeStep * v;      

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

}
