using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class test_joints : MonoBehaviour
{
    // Start is called before the first frame update
    bool addjoint = true;

    float time = 0f;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        time += Time.deltaTime;
        if (time > 10f)
        {
            CharacterJoint[] joints = gameObject.GetComponents<CharacterJoint>();

            foreach(CharacterJoint j in joints)
            {
                Destroy(j);
                Debug.Log("joint delete");
            }
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (addjoint && collision.transform.name!="support")
        {
            AddFixedJoint(collision);
        }

        Debug.Log("on collision enter");
    }

    private void AddFixedJoint(Collision data)
    {
        // if (data.transform.name == "c2")
        // {
        //     FixedJoint joint = gameObject.AddComponent<FixedJoint>();
        //     joint.connectedBody = data.transform.GetComponent<Rigidbody>();
        //     joint.breakForce = Mathf.Infinity;
        // }
        // else
        // {
        //     CharacterJoint joint = gameObject.AddComponent<CharacterJoint>();
        //     joint.connectedBody = data.transform.GetComponent<Rigidbody>();
        //     joint.breakForce = Mathf.Infinity;
        // }
        FixedJoint joint = gameObject.AddComponent<FixedJoint>();
        joint.connectedBody = data.transform.GetComponent<Rigidbody>();
        joint.breakForce = Mathf.Infinity;
        Debug.Log(data.transform.name + " is added to joint " + gameObject.name);
    }
}
