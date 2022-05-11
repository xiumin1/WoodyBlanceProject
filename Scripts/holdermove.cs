using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class holdermove : MonoBehaviour
{
    // Start is called before the first frame update
    // float pos_bounds=0.1f;
    // float ang_bounds=10f;

    Vector3 pos;

    float t;
    void Start()
    {
        pos = this.transform.position;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        t+=Time.deltaTime;
        // if (t>40f)
        // {
        //     t=0f;
        //     float x = MakeRange(pos.x, pos_bounds);
        //     float y = MakeRange(pos.y, pos_bounds);
        //     float z = MakeRange(pos.z, pos_bounds);
        //     this.transform.position = new Vector3(x,y,z);

        //     float rotx = Random.Range(-ang_bounds, ang_bounds);
        //     float roty = Random.Range(-ang_bounds, ang_bounds);
        //     float rotz = Random.Range(-ang_bounds, ang_bounds);
        //     this.transform.eulerAngles = new Vector3(rotx, roty, rotz);
        // }
        
    }

    float MakeRange(float x, float b)
    {
        return Random.Range(x-b, x+b);
    }
}
