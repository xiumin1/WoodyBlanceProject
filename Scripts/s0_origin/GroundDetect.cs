using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class GroundDetect : MonoBehaviour
{

    public WoodyAgent agent1, agent2;
    public Transform board;

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


    }
}
