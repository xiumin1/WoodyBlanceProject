using UnityEngine;

public class GroundDetect_tmhc : MonoBehaviour
{

    public WoodyAgent_tmhc agent1, agent2;
    public Transform board;
    FixedJoint[] joints;
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("tray") || collision.gameObject.CompareTag("ball"))
        {
            agent1.AddReward(-1f);
            agent2.AddReward(-1f);

            agent1.EndEpisode();
            agent2.EndEpisode();
        }
    }

    private void Update()
    {
        var angle = Vector3.Angle(board.up, Vector3.up);
        if (angle > 30.0f)
        {
            agent1.AddReward(-1f);
            agent2.AddReward(-1f);

            agent1.EndEpisode();
            agent2.EndEpisode();
        }

        joints = board.gameObject.GetComponents<FixedJoint>();
        if (joints.Length ==4)
        {
            agent1.AddReward(1f);
            agent2.AddReward(1f);

            agent1.EndEpisode();
            agent2.EndEpisode();
        }
    }
}



