using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoardCollision_menv2 : MonoBehaviour
{
    // Start is called before the first frame update
    WoodyAgent_menv2 agent1, agent2;
    WoodyArea_menv2 area;
    // int terminate_training = 1;

    void Start()
    {
        area = this.GetComponentInParent<WoodyArea_menv2>();
        agent1 = this.transform.parent.Find("Agent1").GetComponent<WoodyAgent_menv2>();
        agent2 = this.transform.parent.Find("Agent2").GetComponent<WoodyAgent_menv2>();
    }
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.name=="pelvis" || collision.gameObject.name=="upper_body")
        {
            // Debug.Log( " Episode ends at " + collision.gameObject.name.ToUpper() + " collide with the board! ");

            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            if (area.multi_env == 1)
            {
                agent1.OnEpisodeBegin();
                agent2.OnEpisodeBegin();
            }
            else
            {
                Debug.Log( " Episode ends at " + collision.gameObject.name.ToUpper() + " collide with the board! " );

                agent1.EndEpisode();
                agent2.EndEpisode();
            }
        }
    }
}
