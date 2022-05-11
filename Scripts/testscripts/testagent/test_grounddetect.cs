using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class test_grounddetect : MonoBehaviour
{

    public test_boardagent agent1, agent2;

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
}
