using UnityEngine;

public class GroundDetect_ikrl : MonoBehaviour
{

    public WoodyAgent_ikrl agent1, agent2;
    public Transform board;
    // FixedJoint[] joints;
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("tray"))
        //  || collision.gameObject.CompareTag("ball"))
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

    // private void FixedUpdate()
    // {
    //     var angle = Vector3.Angle(board.up, Vector3.up);

    //     var dist = Vector3.Distance(board.position, new Vector3(0f,0.96f, 0f));
    //     if (angle > 90.0f || dist>0.5f)
    //     {
    //         // agent1.AddReward(-1f);
    //         // agent2.AddReward(-1f);

    //         agent1.SetReward(-1f);
    //         agent2.SetReward(-1f);

    //         agent1.EndEpisode();
    //         agent2.EndEpisode();

    //         Debug.Log("Episode ends at the board angle greater than 30 degree! ");
    //     }
    // }
}



