using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoardCollision : MonoBehaviour
{
    // Start is called before the first frame update
    public WoodyAgent_mtarget agent1, agent2;
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.name=="pelvis" || collision.gameObject.name=="upper_body")
        {
            // Debug.Log( " Episode ends at " + collision.gameObject.name.ToUpper() + " collide with the board! ");

            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            agent1.EndEpisode();
            agent2.EndEpisode();
        }
    }
}
