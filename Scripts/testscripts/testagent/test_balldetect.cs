using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class test_balldetect : MonoBehaviour
{
    public WoodyArea woodyarea;

    public test_boardagent agent1, agent2;

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
        if (agent1.add_target_angle == 0.0 && agent2.add_target_angle == 0.0)
        {
            if (agent1.countreach == 2 && agent2.countreach == 2)
            {
                agent1.AddReward(1.0f);
                agent2.AddReward(1.0f);
                agent1.EndEpisode();
                agent2.EndEpisode();
            }
        }

        if (agent1.Boardangle() > 45f || agent2.Boardangle() > 45f)
        {
            agent1.AddReward(-1f);
            agent2.AddReward(-1f);

            agent1.EndEpisode();
            agent2.EndEpisode();
        }

        //if (agent1.ConnectBoard() && agent2.ConnectBoard())
        //{
        //    woodyarea.board.GetComponent<Rigidbody>().isKinematic = false;
        //    woodyarea.ball.GetComponent<Rigidbody>().isKinematic = false;
        //}


        if (Vector3.Distance(woodyarea.ball.position, woodyarea.target.position) < 0.05f)
        {

            woodyarea.target.GetComponent<Renderer>().material = crosswin;
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

    // ontriggerstay, still work while the "other" stop moving, and "other" only has limit info 
    //private void OnTriggerStay(Collider other)
    //{
    //    if (other.gameObject.CompareTag("target"))
    //    {
    //        collisiontime += Time.deltaTime;
    //        // check if the ball stay in the target area for more than 3s, if so change the target color to green/wincolor
    //        if (collisiontime > 3f)
    //        {
    //            transform.GetComponent<Renderer>().material = crosswin;
    //            agent1.AddReward(1.0f);
    //            agent2.AddReward(1.0f);
    //            agent1.EndEpisode();
    //            agent2.EndEpisode();
    //            collisiontime = 0f;
    //        }
    //    }


    //}
    /*
    private void OnTriggerExit(Collider other)
    {
        // when the ball is leaving the board
        if (other.CompareTag("tray"))
        {
            agent1.AddReward(-1.0f);
            agent2.AddReward(-1.0f);
            agent1.EndEpisode();
            agent2.EndEpisode();

            //agent1.AddReward(-1f);
            //agent2.AddReward(-1f);

        }
    }
    */

    ////////////////////////////////////////////////////
    // oncollisionstay, only work when "collision" is still moving and colliding with me, and the "collision" has a rich info, like contact points, etc.
    //private void OnCollisionStay(Collision collision)
    //{
    //    if (collision.gameObject.CompareTag("target"))
    //    {
    //        collisiontime += Time.deltaTime;
    //        // check if the ball stay in the target area for more than 3s, if so change the target color to green/wincolor
    //        if (collisiontime > 3f)
    //        {
    //            transform.GetComponent<Renderer>().material = crosswin;
    //        }
    //    }
    //}

    //private void FixedUpdate()
    //{
    //    //Vector3 curv = ballrb.velocity;
    //    //ballrb.velocity = new Vector3(curv.x, 0, curv.y);
    //    //ball.position = new Vector3();
    //    //Debug.DrawRay(ball.position, -board.up * 0.5f, Color.green, Time.deltaTime, false);
        

        
    //}

    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.CompareTag("bound"))
        {
            agent1.AddReward(-0.01f);
            agent2.AddReward(-0.01f);

            //agent1.EndEpisode();
            //agent2.EndEpisode();
        }
    }
}

