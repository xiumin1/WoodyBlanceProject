using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class test_ikrl : MonoBehaviour
{
    // Start is called before the first frame update
    public Rigidbody a1l;
    public Rigidbody a1r;

    public Rigidbody a2l;
    public Rigidbody a2r;


    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
         if (Input.GetKey(KeyCode.Q))
         {
            //  upper_arm_right.AddForce(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            a1l.AddForceAtPosition(Vector3.forward*15f, a1l.position, ForceMode.Force);
            
         }

        if (Input.GetKey(KeyCode.W))
         {
            //  upper_arm_right.AddForce(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            a1r.AddForceAtPosition(Vector3.up*100f, a1r.position, ForceMode.Force);
            
         }

         if (Input.GetKey(KeyCode.E))
         {
            //  upper_arm_right.AddForce(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            a2l.AddForceAtPosition(Vector3.up*1000f, a2l.position, ForceMode.Force);
            
         }

         if (Input.GetKey(KeyCode.R))
         {
            //  upper_arm_right.AddForce(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
            a2r.AddForceAtPosition(Vector3.up*1000f, a2r.position, ForceMode.Force);
            
         }



    }
}
