using UnityEngine;

public class GroundDetect_2acts : MonoBehaviour
{

    public WoodyAgent_2acts agent1, agent2;
    public Transform board;
    // FixedJoint[] joints;
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("tray")  || collision.gameObject.CompareTag("ball"))
        {
            // agent1.AddReward(-1f);
            // agent2.AddReward(-1f);

            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            agent1.EndEpisode();
            agent2.EndEpisode();
            
            Debug.Log("Episode ends at tray or ball collide with the ground! ");
        }
    }
}




