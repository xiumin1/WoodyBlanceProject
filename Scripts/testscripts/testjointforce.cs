using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class testjointforce : MonoBehaviour
{
    // Start is called before the first frame update
    public Rigidbody shoulder;

    public Rigidbody elbow;

    public Rigidbody hand;

    public float x = 10, y = 10, z = 10;

    bool connect = false;

    public GameObject lefthold;
    public Rigidbody lefthand;
    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.Q)) shoulder.AddTorque(new Vector3(x, 0, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.A)) shoulder.AddTorque(new Vector3(-x, 0, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.W)) shoulder.AddTorque(new Vector3(0, y, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.S)) shoulder.AddTorque(new Vector3(0, -y, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.E)) shoulder.AddTorque(new Vector3(0, 0, z), ForceMode.Force);

        if (Input.GetKey(KeyCode.D)) shoulder.AddTorque(new Vector3(0, 0, -z), ForceMode.Force);

        if (Input.GetKey(KeyCode.T)) elbow.AddTorque(new Vector3(x, 0, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.G)) elbow.AddTorque(new Vector3(-x, 0, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.Y)) elbow.AddTorque(new Vector3(0, y, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.H)) elbow.AddTorque(new Vector3(0, -y, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.U)) elbow.AddTorque(new Vector3(0, 0, z), ForceMode.Force);

        if (Input.GetKey(KeyCode.J)) elbow.AddTorque(new Vector3(0, 0, -z), ForceMode.Force);

        if (Input.GetKey(KeyCode.Z)) hand.AddTorque(new Vector3(x, 0, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.X)) hand.AddTorque(new Vector3(-x, 0, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.C)) hand.AddTorque(new Vector3(0, y, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.V)) hand.AddTorque(new Vector3(0, -y, 0), ForceMode.Force);

        if (Input.GetKey(KeyCode.B)) hand.AddTorque(new Vector3(0, 0, z), ForceMode.Force);

        if (Input.GetKey(KeyCode.N)) hand.AddTorque(new Vector3(0, 0, -z), ForceMode.Force);


        if (x==11 && !connect)
        {
            connect = true;
            lefthold.AddComponent<CharacterJoint>();
            CharacterJoint joint = lefthold.GetComponent<CharacterJoint>();
            Rigidbody rdb = lefthold.GetComponent<Rigidbody>();
            rdb.isKinematic = true;
            joint.connectedBody = lefthand;

        }

        if (x == 12 && connect)
        {
            connect = false;

        }
    }
}
