using UnityEngine;

public class GroundDetect_mtarget : MonoBehaviour
{

    public WoodyAgent_mtarget agent1, agent2;
    public Transform board;
    // FixedJoint[] joints;
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.name=="Board"  || collision.gameObject.CompareTag("ball"))
        // if (collision.gameObject.CompareTag("ball")) 
        {
            // Debug.Log( " Episode ends at " + collision.gameObject.name.ToUpper() + " collide with the ground! " );//+ collision.gameObject.transform.position.ToString("F3") + this.transform.position);

            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            agent1.EndEpisode();
            agent2.EndEpisode();
        }
    }
}




