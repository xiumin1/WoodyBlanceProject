using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class testnormal : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject board;
    float angledifference;
    Vector3 boardnormal;
    Vector3 groundnormal;

    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        /*
        groundnormal = Vector3.up;
        boardnormal = board.transform.up;

        angledifference = Vector3.Distance(groundnormal, boardnormal);
        Debug.Log("groundnormal= " + groundnormal.ToString("F4") + " boardnormal= " + boardnormal.ToString("F4") + " difference= " + angledifference.ToString("F4"));
        Debug.Log("boardnormal normalized= " + boardnormal.normalized.ToString("F4"));
        */
        Vector3 v1 = new Vector3(1, 1, 0);
        Vector3 v2 = new Vector3(2, 1, 0);
        Vector3 v3 = new Vector3(3, 4, 0);

        float angle = Vector3.Angle(v1, v2);
        Debug.Log("angle difference: " + angle);
        Debug.DrawLine(v1, v1 + v2, Color.green, 2, false);
        Debug.DrawRay(v1, v1+v2, Color.red, 2, false);

    }
}
