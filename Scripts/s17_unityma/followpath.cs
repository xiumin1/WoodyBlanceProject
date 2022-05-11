using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class followpath : MonoBehaviour
{
    // Start is called before the first frame update
    float pathx, pathy, pathz, rad;
    float speed = 0.5f;

    public Transform target;

    int episode_step_counter = 0;

    void Start()
    {
        
        speed *=1f;
        episode_step_counter *=1;

        initparams();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        episode_step_counter += 1;

        if(episode_step_counter > 500)
        {
            initparams();
            episode_step_counter =0;
        }

        rad += speed * Time.deltaTime;
        float x = this.transform.localPosition.x + pathx * Mathf.Cos(rad);
        float z = this.transform.localPosition.z + pathz * Mathf.Sin(rad);
        float y = this.transform.localPosition.y + pathy * Mathf.Cos(rad);
        // target.position = new Vector3(x,y,z);
        target.position = new Vector3(x, y, z);
        // target.position = this.transform.TransformPoint(new Vector3(x,y,z));
        
    }

    void initparams()
    {
        pathx = Random.Range(0, 0.1f);
        pathy = Random.Range(0, 0.1f);
        pathz = Random.Range(0, 0.1f);
        rad = Random.Range(0,7);
    }
}
