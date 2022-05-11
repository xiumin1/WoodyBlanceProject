using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateCam : MonoBehaviour
{
    // Start is called before the first frame update
    float cam_degree = 90;
    float cam_deviation = 0;
    float cam_deviationfreq = 0;
    public float speed = 0.1f;
    public float lookdistance = 2.5f;
    // Update is called once per frame
    public WoodyAgent agent;

    //private void Start()
    //{
    //    speed = agent.cam_rot_speed;
    //    lookdistance = agent.cam_look_distance;
    //}
    void Update()
    {
        
        //transform.Rotate(0, rot++, 0);
        var radians = cam_degree * Mathf.PI / 180f;
        
        var camX = lookdistance * Mathf.Sin(radians);
        var camY= cam_deviation * Mathf.Cos(cam_deviationfreq * radians);
        var camZ = lookdistance * Mathf.Cos(radians);

        transform.position = new Vector3(camX, transform.position.y, camZ);// + transform.position;
        transform.LookAt(new Vector3(0,1f,0));

        cam_degree += speed;
    }


}
