using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class supportfollow : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform parent;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        var x = parent.position.x;
        var z = parent.position.z;
        this.transform.position = new Vector3(x, this.transform.position.y, z);
    }
}
