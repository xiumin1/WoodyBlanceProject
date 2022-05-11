using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class BallDetect_hc : MonoBehaviour
{
    public WoodyArea woodyarea;

    public WoodyAgent_hc agent1, agent2;

    private Material crosswin;
    private Material crosslose;

    private Renderer rendermat;

    float collisiontime = 0.0f;
    // Start is called before the first frame update
    void Start()
    {
        crosswin = woodyarea.crosswin;
        crosslose = woodyarea.crosslose;
        rendermat = woodyarea.target.GetComponent<Renderer>();
        //myconsole = this.transform.GetComponent<Console>();
    }

    private void Update()
    {

        if (Vector3.Distance(woodyarea.ball.position, woodyarea.target.position) < 0.05f)
        {
            rendermat.material = crosswin;
            collisiontime += Time.deltaTime;
            // check if the ball stay in the target area for more than 3s, if so change the target color to green/wincolor
            if (collisiontime > 3f)
            {
                agent1.AddReward(1.0f);
                agent2.AddReward(1.0f);
                agent1.EndEpisode();
                agent2.EndEpisode();
                collisiontime = 0f;
            }
        }
        else
        {
            rendermat.material = crosslose;
            collisiontime = 0f;
        }
    }
}
