using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class CarEngine : MonoBehaviour {

    public Transform path;
    public float maxSteerAngle = 45f;
    public WheelCollider wheelFL;
    public WheelCollider wheelFR;
    public float maxMotorTorque = 80f;
    public float currentSpeed;
    public float maxSpeed = 100f;
    public Vector3 centerOfMass; 

    private List<Transform> nodes;
    private int currentNode = 0;
    

        void Start () {
        GetComponent<Rigidbody>().centerOfMass = centerOfMass;
        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();

        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }
    }
	
    private void FixedUpdate()
    {
        ApplySteer();
        Drive();
        CheckWaypointDistance();
    }
	
    private void ApplySteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(nodes[currentNode].position);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngle;
        wheelFL.steerAngle = newSteer;
        wheelFR.steerAngle = newSteer;
    }

    private void Drive()
    {
        currentSpeed = 2 * Mathf.PI * wheelFL.radius * wheelFL.rpm * 60 / 100;
        if(currentSpeed < maxSpeed)
        {
            wheelFR.motorTorque = maxMotorTorque;
            wheelFL.motorTorque = maxMotorTorque;
        }
        else
        {
            wheelFR.motorTorque = 0;
            wheelFL.motorTorque = 0;
        }
        
    }

    private void CheckWaypointDistance()
    {
        if(Vector3.Distance(transform.position, nodes[currentNode].position) < 0.5f){
            if(currentNode == nodes.Count - 1)
            {
                currentNode = 0;
            }
            else
            {
                currentNode++; 
            }
        }
    }
}
