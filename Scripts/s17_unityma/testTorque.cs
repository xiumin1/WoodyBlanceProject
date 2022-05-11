using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class testTorque : MonoBehaviour
{
    // Start is called before the first frame update
    Rigidbody rb;
    public Vector3 torque;
    void Start()
    {
        rb = this.transform.GetComponent<Rigidbody>();
        torque = new Vector3(1,0,0);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        rb.AddRelativeTorque(torque, ForceMode.Force);
        // rb.AddTorque(torque, ForceMode.Force);
    }
}
