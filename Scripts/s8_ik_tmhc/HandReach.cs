using UnityEngine;

public class HandReach : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform left_final_target, left_ik_target, right_final_target, right_ik_target;
    public Transform board;
    public Transform left_hand, right_hand;
    float left_journeyLength=0f, right_journeyLength = 0f;
    float startTime;
    float speed = 0.5f;

    public bool left_connect = false, right_connect=false;
    float threshold = 0.001f;

    Vector3 left_handpos, right_handpos;
    Quaternion left_handrot, right_handrot;

    float left_journeycheck = 1f, right_journeycheck = 1f;
    void Start()
    {
        startTime = Time.time;
        left_journeyLength = Vector3.Distance(left_ik_target.position, left_final_target.position);
        left_handpos = left_hand.position;
        left_handrot = left_hand.rotation;

        right_journeyLength = Vector3.Distance(right_ik_target.position, right_final_target.position);
        right_handpos = right_hand.position;
        right_handrot = right_hand.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        if (left_journeycheck > threshold || right_journeycheck > threshold)
        {
            left_journeycheck = HandMove(left_handrot, left_handpos, left_journeyLength, left_ik_target, left_final_target);
            right_journeycheck = HandMove(right_handrot, right_handpos, right_journeyLength, right_ik_target, right_final_target);
        }
        else
        {
            if(!left_connect)
            {
                FixedJoint joint = board.gameObject.AddComponent<FixedJoint>();
                joint.connectedBody = left_hand.transform.GetComponent<Rigidbody>();
                joint.breakForce = Mathf.Infinity;

                left_connect = true;
                // Debug.Log("left joint connect 1111111111111111111111111"); 
            }
            if(!right_connect)
            {
                FixedJoint joint = board.gameObject.AddComponent<FixedJoint>();
                joint.connectedBody = right_hand.transform.GetComponent<Rigidbody>();
                joint.breakForce = Mathf.Infinity;

                right_connect = true;
                // Debug.Log("right joint connect 2222222222222222222222222222");
            }
        }
    }

    float HandMove(Quaternion handrot, Vector3 handpos, float journeyLength, Transform ik_target, Transform final_target)
    {
        var distJourney = (Time.time - startTime) * speed;
        var fracJourney = Mathf.Clamp(distJourney / journeyLength, 0, 1);
        // fracJourney = -2 * Mathf.Pow(fracJourney, 3.0f) + 3 * Mathf.Pow(fracJourney, 2.0f);//

        ik_target.rotation = Quaternion.Lerp(handrot, final_target.rotation, fracJourney);
        ik_target.position = Vector3.Lerp(handpos, final_target.position, fracJourney);

        var journeycheck = Mathf.Abs(fracJourney - 1f);
        return journeycheck;
    }

    void OnDisable()
    {
        left_connect = false;
        right_connect = false;

        left_journeycheck = 1f; 
        right_journeycheck = 1f;
    }

    void OnEnable()
    {
        startTime = Time.time;
    }
}
