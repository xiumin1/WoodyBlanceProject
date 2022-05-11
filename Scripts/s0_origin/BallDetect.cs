using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class BallDetect : MonoBehaviour
{
    public WoodyArea woodyarea;

    public WoodyAgent agent1, agent2;

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
    
        //if (agent1.use_target_angle == 0.0 && agent2.use_target_angle == 0.0)
        //{
        //    if (agent1.countreach == 2 && agent2.countreach == 2)
        //    {
        //        agent1.AddReward(1.0f);
        //        agent2.AddReward(1.0f);
        //        agent1.EndEpisode();
        //        agent2.EndEpisode();
        //    }
        //}
        ////else if (agent1.add_target_angle == 1.0 && agent2.add_target_angle == 1.0)
        ////{
        ////    if (agent1.add_lifting_board == 0.0 && agent2.add_lifting_board == 0.0)
        ////    {
        ////        if (agent1.countreach == 6 && agent2.countreach == 6)
        ////        {
        ////            agent1.AddReward(1.0f);
        ////            agent2.AddReward(1.0f);
        ////            agent1.EndEpisode();
        ////            agent2.EndEpisode();
        ////        }
        ////    }
        ////    //else if (agent1.add_lifting_board == 1.0 && agent2.add_lifting_board == 1.0)
        ////    //{
        ////    //    if (agent1.countreach == 8 && agent2.countreach == 8)
        ////    //    {
        ////    //        agent1.AddReward(1.0f);
        ////    //        agent2.AddReward(1.0f);
        ////    //        agent1.EndEpisode();
        ////    //        agent2.EndEpisode();
        ////    //    }
        ////    //}
        ////}


        //if (agent1.ConnectBoard() && agent2.ConnectBoard())
        //{
        //    woodyarea.board.GetComponent<Rigidbody>().isKinematic = false;
        //    woodyarea.ball.GetComponent<Rigidbody>().isKinematic = false;
        //}


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

        if (woodyarea.support.activeSelf == true)
        {
            if (woodyarea.board.position.y > woodyarea.y_target + 0.05f)
            {
                woodyarea.support.SetActive(false);
                agent1.AddReward(0.5f);
                agent2.AddReward(0.5f);
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
}
