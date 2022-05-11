using UnityEngine;

public class GroundDetect_menv : MonoBehaviour
{

    WoodyAgent_menv agent1, agent2;
    WoodyArea_menv area;
    Transform board, ball;
    float checkboardreachtime = 0;
    // int terminate_training = 1; // 1 means terminate for training purpose, 16 environments; 0 means terminate for testing purpose, single environment
    // FixedJoint[] joints;
    void Start()
    {
        area = this.GetComponentInParent<WoodyArea_menv>();
        agent1 = this.transform.parent.Find("Agent1").GetComponent<WoodyAgent_menv>();
        agent2 = this.transform.parent.Find("Agent2").GetComponent<WoodyAgent_menv>();
        board = this.transform.parent.Find("Board");
        ball = this.transform.parent.Find("Ball");
    }
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.name=="Board"  || collision.gameObject.CompareTag("ball"))
        // if (collision.gameObject.CompareTag("ball")) 
        {
            //+ collision.gameObject.transform.position.ToString("F3") + this.transform.position);

            agent1.SetReward(-1f);
            agent2.SetReward(-1f);

            // agent1.handreach_reward = -1f;
            // agent1.boardreach_reward = -1f;
            // agent1.ballreach_reward = -1f;

            // agent2.handreach_reward = -1f;
            // agent2.boardreach_reward = -1f;
            // agent2.ballreach_reward = -1f;
            if (area.multi_env==1)
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
        var agent1_dist = Vector3.Distance(agent1.ar1.position, agent1.r1.position) + Vector3.Distance(agent1.al1.position, agent1.l1.position);
        var agent2_dist = Vector3.Distance(agent2.ar1.position, agent2.r1.position) + Vector3.Distance(agent2.al1.position, agent2.l1.position);

        if(agent1_dist>1 || agent2_dist>1)
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
                Debug.Log( " Episode ends at agent hands distance to board more than 1 " );
                
                agent1.EndEpisode();
                agent2.EndEpisode();
            }
        }

        if((agent1.count_steps > agent1.terminate_steps) || (agent2.count_steps > agent2.terminate_steps))
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
                Debug.Log( " Episode ends at max episode steps " + agent1.count_steps );
                
                agent1.EndEpisode();
                agent2.EndEpisode();
            }

        }

        if(Vector3.Distance(board.position, this.transform.position) > 2f || Vector3.Distance(board.position, ball.position)>0.6f)
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
                Debug.Log( " Episode ends at board out of environment range or ball goes out of board!" );
                
                agent1.EndEpisode();
                agent2.EndEpisode();
            }
        }

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

        if(agent1.count_steps>100 && (agent1.boardreach_reward < 0.85f || agent2.boardreach_reward < 0.85f))
        {
            checkboardreachtime += Time.deltaTime;
            if(checkboardreachtime>2f)
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
                    Debug.Log( "low reward for borad reach exceeds 3s! ");
                    
                    agent1.EndEpisode();
                    agent2.EndEpisode();
                }
            }
        }
        else
        {
            checkboardreachtime = 0f;
        }

    }
}




