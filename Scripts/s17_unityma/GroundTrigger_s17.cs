using UnityEngine;
using UnityEngine.Events;

public class GroundTrigger_s17 : MonoBehaviour
{

    [System.Serializable]
    public class CollisionEventGround : UnityEvent< float>
    {
    }

    [Header("Collision Callbacks")]
    public CollisionEventGround onCollisionEnterEvent = new CollisionEventGround();
    // public CollisionEventGround onCollisionStayEvent = new CollisionEventGround();
    // public CollisionEventGround onCollisionExitEvent = new CollisionEventGround();

    private void OnCollisionEnter(Collision col)
    {
        if (col.gameObject.name =="Board" || col.gameObject.name == "Ball")
        {
            Debug.Log("terminate becuase ground hit " + col.gameObject.name);
            onCollisionEnterEvent.Invoke(1f);
            Debug.Log("terminate becuase ground hit " + col.gameObject.name);
        }
    }

}




