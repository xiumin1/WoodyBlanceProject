using UnityEngine;

public class GroundDetect_menv2 : MonoBehaviour
{

    WoodyAgent_menv2 agent1, agent2;
    WoodyArea_menv2 area;
    Transform board, ball;
    // int terminate_training = 1; // 1 means terminate for python training with multiple environment, 0 means terminate for python testing single environment
    // FixedJoint[] joints;
    // float checkboardreachtime = 0, checkhandreachtime = 0;

    // float temptime = 0;
    void Start()
    {
        area = this.GetComponentInParent<WoodyArea_menv2>();
        agent1 = this.transform.parent.Find("Agent1").GetComponent<WoodyAgent_menv2>();
        agent2 = this.transform.parent.Find("Agent2").GetComponent<WoodyAgent_menv2>();
        board = this.transform.parent.Find("Board");
        ball = this.transform.parent.Find("Ball");
    }
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.name=="Board"  || collision.gameObject.CompareTag("ball"))
        // if (collision.gameObject.CompareTag("ball")) 
        {
            // agent1.AddReward(-1f);
            // agent2.AddReward(-1f);

            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            if (area.multi_env == 1)
            {
                agent1.OnEpisodeBegin();
                agent2.OnEpisodeBegin();
            }
            else
            {
                Debug.Log( " Episode ends at " + collision.gameObject.name.ToUpper() + " collide with the ground! " );

                agent1.EndEpisode();
                agent2.EndEpisode();
            }
            
        }
    }

    private void FixedUpdate()
    {
        // check the the training exceeds the terminate steps that allowed in an episode
        // temptime += Time.deltaTime;
        if((agent1.count_steps > agent1.terminate_steps) || (agent2.count_steps > agent2.terminate_steps))
        {
            // Debug.Log("terminate steps time cost= " + temptime);
            // temptime = 0;

            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            if (area.multi_env == 1)
            {
                agent1.OnEpisodeBegin();
                agent2.OnEpisodeBegin();
            }
            else
            {
                Debug.Log( " Episode ends at max episode steps " + agent1.count_steps );
                
                agent1.EndEpisode();
                agent2.EndEpisode();
            }
        }
        // check if the board go out of the area range
        float distx = Mathf.Abs(board.position.x - area.transform.position.x);
        float distz = Mathf.Abs(board.position.z - area.transform.position.z);
        float disty = Mathf.Abs(board.position.y - area.transform.position.y);

        if (distx>1f || distz > 1f || disty>1.5f || Vector3.Distance(board.position, ball.position)>0.6f)
        {
            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            if (area.multi_env == 1)
            {
                agent1.OnEpisodeBegin();
                agent2.OnEpisodeBegin();
            }
            else
            {
                Debug.Log( " Episode ends at going out of area range! ");
                
                agent1.EndEpisode();
                agent2.EndEpisode();
            }
        }

        // terminate if board tilt more than 30 degree
        var boardang = Vector3.Angle(board.up, Vector3.up);
        if(boardang>30f)
        {
            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            if (area.multi_env == 1)
            {
                agent1.OnEpisodeBegin();
                agent2.OnEpisodeBegin();
            }
            else
            {
                Debug.Log( " Episode ends at board tilt more than 30 degree! ");
                
                agent1.EndEpisode();
                agent2.EndEpisode();
            }
        }

        // if(agent1.count_steps>100 && (agent1.boardreach_reward < 0.85f || agent2.boardreach_reward < 0.85f) )
        // {
        //     checkboardreachtime += Time.deltaTime;
        //     if(checkboardreachtime>2f)
        //     {
        //         agent1.SetReward(-1f);
        //         agent2.SetReward(-1f);

        //         if (area.multi_env == 1)
        //         {
        //             agent1.OnEpisodeBegin();
        //             agent2.OnEpisodeBegin();
        //         }
        //         else
        //         {
        //             Debug.Log( "low reward for borad reach exceeds 3s! ");
                    
        //             agent1.EndEpisode();
        //             agent2.EndEpisode();
        //         }
        //     }
        // }
        // else
        // {
        //     checkboardreachtime = 0f;
        // }

        // if(agent1.count_steps>100 && (agent1.handreach_reward < 0.8f || agent2.handreach_reward<0.8f))
        // {
        //     checkhandreachtime += Time.deltaTime;
        //     if(checkhandreachtime >2f)
        //     {
        //         agent1.SetReward(-1f);
        //         agent2.SetReward(-1f);

        //         if (area.multi_env == 1)
        //         {
        //             agent1.OnEpisodeBegin();
        //             agent2.OnEpisodeBegin();
        //         }
        //         else
        //         {
        //             Debug.Log( "low reward for hand reach exceeds 3s! ");
                    
        //             agent1.EndEpisode();
        //             agent2.EndEpisode();
        //         }
        //     }
        // }
        // else
        // {
        //     checkhandreachtime = 0f;
        // }
    }
}




