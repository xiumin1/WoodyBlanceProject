using System.Collections;
using System.Collections.Generic;
using UnityEngine;
 
public class MouseInstantiate : MonoBehaviour
{
    GameObject prefabToInstantiate;
    Vector3 origin;
    public Transform board;
    GameObject newGameObject;

    void Start()
    {
        prefabToInstantiate = Resources.Load<GameObject>("Prefabs/Sphere");
    }
    public void Update()
    {
        if (prefabToInstantiate == null)
        {
            return;
        }
 
        if (Input.GetMouseButtonDown(0))
        {
            var x = Random.Range(board.position.x - 0.3f, board.position.x+0.3f);
            var z = Random.Range(board.position.z - 0.3f, board.position.z+0.3f);
            origin = new Vector3(x, 1.5f, z);

            if (newGameObject !=null)
                Destroy(newGameObject);
            else
                newGameObject = (GameObject)Instantiate(prefabToInstantiate, origin, Quaternion.identity);

        }
    }
}