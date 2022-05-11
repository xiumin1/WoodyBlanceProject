using UnityEngine;
using UnityEngine.Events;

public class BoardTrigger_s17 : MonoBehaviour
{

    [System.Serializable]
    public class CollisionEventBoard : UnityEvent< float >
    {
    }

    [Header("Collision Callbacks")]
    public CollisionEventBoard onCollisionEnterEvent = new CollisionEventBoard();
    // public CollisionEventBoard onCollisionStayEvent = new CollisionEventBoard();
    // public CollisionEventBoard onCollisionExitEvent = new CollisionEventBoard();

    private void OnCollisionEnter(Collision col)
    {
        if (col.gameObject.name=="pelvis" || col.gameObject.name=="upper_body")
        {
            onCollisionEnterEvent.Invoke(1f);
            
            Debug.Log("terminate becuase board hit " + col.gameObject.name);  
            
        }
    }

}




