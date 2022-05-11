using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class test_ballsingle : MonoBehaviour
{
    public WoodyArea woodyarea;

    public test_agentsingle agent;

    private Material crosswin;
    private Material crosslose;

    Rigidbody ballrb;
    Transform ball;
    Transform board;

    float collisiontime = 0.0f;

    Vector3 pos;
    float dist;
    RaycastHit hit;
    // Start is called before the first frame update
    void Start()
    {
        crosswin = woodyarea.crosswin;
        crosslose = woodyarea.crosslose;

        pos = woodyarea.board.transform.position;
        ballrb = this.GetComponent<Rigidbody>();
        ball = this.transform;
        board = woodyarea.board;

        dist = (ball.lossyScale.y + board.lossyScale.y) / 2f;
        //myconsole = this.transform.GetComponent<Console>();
    }

    private void FixedUpdate()
    {
        //woodyarea.board.position = new Vector3(pos.x, 0.965f, pos.z);
        if (agent.add_target_angle == 0.0 )
        {
            if (agent.countreach == 2)
            {
                agent.AddReward(1.0f);
                
                agent.EndEpisode();
            }
        }

        if (agent.Boardangle() > 45f)
        {
            agent.AddReward(-1f);
            agent.EndEpisode();
        }

        if (Vector3.Distance(woodyarea.ball.position, woodyarea.target.position) < 0.05f)
        {

            woodyarea.target.GetComponent<Renderer>().material = crosswin;
            collisiontime += Time.deltaTime;
            // check if the ball stay in the target area for more than 3s, if so change the target color to green/wincolor
            if (collisiontime > 3f)
            {
                agent.AddReward(1.0f);
                
                agent.EndEpisode();
                collisiontime = 0f;
            }
        }
        else
        {
            woodyarea.target.GetComponent<Renderer>().material = crosslose;
            collisiontime = 0f;
        }

        // to make sure the ball will not go through the board becuase of the physics check time different from the frame rate
        if (Physics.Raycast(ball.position, -board.up, out hit, Mathf.Infinity) && hit.collider.tag == "tray")
        {

            if (hit.distance > dist * 1.1f || hit.distance < dist * 0.9f)
            {
                ballrb.MovePosition(ball.position + (-board.up) * (hit.distance - dist));
            }

        }
        else
        {
            if (Physics.Raycast(ball.position, board.up, out hit, Mathf.Infinity) && hit.collider.tag == "tray")
            {
                ballrb.MovePosition(ball.position + (board.up) * (hit.distance + dist));
            }
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.CompareTag("bound"))
        {
            agent.AddReward(-0.01f);
         
        }
    }
}


