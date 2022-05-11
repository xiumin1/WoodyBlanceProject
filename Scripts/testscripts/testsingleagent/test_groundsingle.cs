using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class test_groundsingle : MonoBehaviour
{

    public test_agentsingle agent;

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("tray") || collision.gameObject.CompareTag("ball"))
        {
            agent.AddReward(-1f);
           
            agent.EndEpisode();

        }
    }
}
