
using UnityEngine;

public class BallDetect_ikrl : MonoBehaviour
{
    public WoodyArea_ik woodyarea;
    public WoodyAgent_ikrl agent1, agent2;
    private Material crosswin;
    private Material crosslose;
    private Renderer rendermat;
    float collisiontime = 0.0f;
    // Start is called before the first frame update

    Rigidbody ballrigid;
    Rigidbody boardrigid;
    void Start()
    {
        crosswin = woodyarea.crosswin;
        crosslose = woodyarea.crosslose;
        rendermat = woodyarea.target.GetComponent<Renderer>();

        ballrigid = this.GetComponent<Rigidbody>();
        boardrigid = woodyarea.board.GetComponent<Rigidbody>();
    }
    private void FixedUpdate()
    {
        if (Vector3.Distance(woodyarea.ball.position, woodyarea.target.position) < 0.05f)
        {
            rendermat.material = crosswin;
            collisiontime += Time.deltaTime;
            // check if the ball stay in the target area for more than 3s, if so change the target color to green/wincolor
            if (collisiontime >= agent1.target_maintain_time)
            {
            //    agent1.AddReward(1.0f);
            //    agent2.AddReward(1.0f);

               agent1.SetReward(1f);
               agent2.SetReward(1f);
               agent1.EndEpisode();
               agent2.EndEpisode();
               collisiontime = 0f;

               Debug.Log("Episode ends at the final 3s success target hits! ");
            }
            // agent1.AddReward(1.0f);
            // agent2.AddReward(1.0f);
            // agent1.EndEpisode();
            // agent2.EndEpisode();
        }
        else
        {
            rendermat.material = crosslose;
            collisiontime = 0f;
        }

        // if (woodyarea.support.activeSelf == true)
        // {
        //     if (woodyarea.board.position.y > woodyarea.y_target + 0.05f)
        //     {
        //         woodyarea.support.SetActive(false);
        //         agent1.AddReward(0.5f);
        //         agent2.AddReward(0.5f);
        //     }
        // }
    }

}


