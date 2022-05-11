using UnityEngine;

public class RotateCam_s17 : MonoBehaviour
{
    // Start is called before the first frame update
    float cam_degree = 90;
    float cam_deviation = 0;
    float cam_deviationfreq = 0;
    // Update is called once per frame
    public float cam_look_distance = 1.5f; 
    public float cam_rot_speed = 0.1f;
    void Update()
    {
        var radians = cam_degree * Mathf.PI / 180f;

        var camX = cam_look_distance * Mathf.Sin(radians);
        var camY = cam_deviation * Mathf.Cos(cam_deviationfreq * radians);
        var camZ = cam_look_distance * Mathf.Cos(radians);

        transform.position = new Vector3(camX, transform.position.y, camZ);// + transform.position;
        transform.LookAt(new Vector3(0, 1f, 0));

        cam_degree += cam_rot_speed;
    }
}

