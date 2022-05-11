using UnityEngine;

public class RotateCam_s13 : MonoBehaviour
{
    // Start is called before the first frame update
    float cam_degree = 90;
    float cam_deviation = 0;
    float cam_deviationfreq = 0;
    public float speed = 0.1f;
    public float lookdistance = 2.5f;
    // Update is called once per frame
    public WoodyAgent_s13 agent;

    void Update()
    {
        var radians = cam_degree * Mathf.PI / 180f;

        var camX = agent.cam_look_distance * Mathf.Sin(radians);
        var camY = cam_deviation * Mathf.Cos(cam_deviationfreq * radians);
        var camZ = agent.cam_look_distance * Mathf.Cos(radians);

        transform.position = new Vector3(camX, transform.position.y, camZ);// + transform.position;
        transform.LookAt(new Vector3(0, 1f, 0));

        cam_degree += agent.cam_rot_speed;

    }
}

