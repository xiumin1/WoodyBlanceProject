using UnityEngine;

public class RotateCam_menv : MonoBehaviour
{
    // Start is called before the first frame update
    float cam_degree = 90;
    float cam_deviation = 0;
    float cam_deviationfreq = 0;
    public float speed = 0.1f;
    public float lookdistance = 2.5f;

    public Vector3 cam_rel_pos;
    void Update()
    {
        var radians = cam_degree * Mathf.PI / 180f;

        var camX = lookdistance * Mathf.Sin(radians);
        var camY = cam_deviation * Mathf.Cos(cam_deviationfreq * radians);
        var camZ = lookdistance * Mathf.Cos(radians);

        transform.position = new Vector3(camX + cam_rel_pos.x, transform.position.y, camZ + cam_rel_pos.z);// + transform.position;
        transform.LookAt(cam_rel_pos);

        cam_degree += speed;
    }
}

