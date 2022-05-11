using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class test : MonoBehaviour
{
    // Start is called before the first frame update

    //public Transform rhand;
    //public Transform rhand1;
    //public Transform lhand;
    //public Transform lhand1;


    //float rjourneyLength;
    //float distJourney;
    //float rfracJourny;

    //float ljourneyLength;

    //float lfracJourny;

    //float startTime;
    //float speed = 1.0f;

    public Rigidbody lefthold, righthold;
    public Rigidbody board;
    public Rigidbody ball;
    float act1, act2;
    public float ratio = 10f;

    //void Start()
    //{
    //    startTime = Time.time;

        

    //    rjourneyLength = Vector3.Distance(rhand.position, rhand1.position);
        

    //    ljourneyLength = Vector3.Distance(lhand.position, lhand1.position);
        

    //}

    // Update is called once per frame
    void Update()
    {
        //distJourney = (Time.time - startTime) * speed;

        //rfracJourny = distJourney / rjourneyLength;
        //lfracJourny = distJourney / ljourneyLength;



        if (Input.GetKey(KeyCode.Q)) { act1 = ratio; lefthold.AddForce(new Vector3(0, act1, 0), ForceMode.Force); }

        else if (Input.GetKey(KeyCode.A)) { act1 = -ratio; lefthold.AddTorque(new Vector3(0, act1, 0), ForceMode.Acceleration); }

        else if (Input.GetKey(KeyCode.W)) { act2 = ratio; righthold.AddTorque(new Vector3(act2, 0, 0f), ForceMode.Acceleration); }

        else if (Input.GetKey(KeyCode.S)) { act2 = -ratio; righthold.AddTorque(new Vector3(act2, 0, 0f), ForceMode.Acceleration); }

        else { act1 = 0; act2 = 0f; lefthold.AddTorque(new Vector3(0f, act1, 0f), ForceMode.VelocityChange); righthold.AddTorque(new Vector3(0f, act2, 0f), ForceMode.VelocityChange); }


        //if (Input.GetKey(KeyCode.Q)){act1 = ratio; board.AddForceAtPosition(new Vector3(0f, act1, 0f) , lefthold.position, ForceMode.Acceleration);}

        //else if (Input.GetKey(KeyCode.A)){act1 = -ratio; board.AddForceAtPosition(new Vector3(0f, act1, 0f) , lefthold.position, ForceMode.Acceleration);}

        //else if (Input.GetKey(KeyCode.W)) { act2 = ratio; board.AddForceAtPosition(new Vector3(0f, act2, 0f) , righthold.position, ForceMode.Acceleration); }

        //else if (Input.GetKey(KeyCode.S)) { act2 = -ratio; board.AddForceAtPosition(new Vector3(0f, act2, 0f) , righthold.position, ForceMode.Acceleration); }

        //else { act1 = 0;  act2 = 0f; board.AddForceAtPosition(new Vector3(0f, act1, 0f) , righthold.position, ForceMode.VelocityChange); board.AddForceAtPosition(new Vector3(0f, act2, 0f), righthold.position, ForceMode.VelocityChange); }

        //if rhand
        //Debug.Log("act1= " + act1 + " act2= " + act2);
        //board.AddForceAtPosition(new Vector3(0f, act1, 0f)*5, lefthold.position, ForceMode.Force);
        //board.AddForceAtPosition(new Vector3(0f, act2, 0f)*5, righthold.position, ForceMode.Force);
    }

    
}
