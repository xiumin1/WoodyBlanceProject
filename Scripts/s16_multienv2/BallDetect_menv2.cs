
using UnityEngine;

public class BallDetect_menv2 : MonoBehaviour
{
    WoodyArea_menv2 woodyarea;
    WoodyAgent_menv2 agent1, agent2;
    private Material crosswin;
    private Material crosslose;
    private Renderer rendermat;
    float collisiontime = 0.0f;
    // Start is called before the first frame update
    
    Rigidbody ballrigid;
    Rigidbody boardrigid;
    void Start()
    {
        agent1 = this.transform.parent.Find("Agent1").GetComponent<WoodyAgent_menv2>();
        agent2 = this.transform.parent.Find("Agent2").GetComponent<WoodyAgent_menv2>();
        woodyarea = this.GetComponentInParent<WoodyArea_menv2>();

        crosswin = woodyarea.crosswin;
        crosslose = woodyarea.crosslose;
        rendermat = woodyarea.target.GetComponent<Renderer>();

        // rendermat = woodyarea.target.GetChild(0).GetComponent<Renderer>();

        ballrigid = this.GetComponent<Rigidbody>();
        boardrigid = woodyarea.board.GetComponent<Rigidbody>();
    }
    private void FixedUpdate()
    {
        // Debug.Log("ball target distance: " + Vector3.Distance(woodyarea.ball.position, agent1.target.position));
        if (Vector3.Distance(woodyarea.ball.position, agent1.target.position) < 0.02f)
        {
            rendermat.material = crosswin;
            collisiontime += Time.deltaTime;
            // check if the ball stay in the target area for more than 3s, if so change the target color to green/wincolor
            if (collisiontime >= agent1.target_maintain_time)
            {
            //    agent1.AddReward(1.0f);
            //    agent2.AddReward(1.0f);
                collisiontime = 0f; 
                agent1.SetReward(1f);
                agent2.SetReward(1f);
               if (woodyarea.multi_env == 1)
                {
                    agent1.OnEpisodeBegin();
                    agent2.OnEpisodeBegin();
                }
                else
                {
                    Debug.Log( " Episode ends at target position maintained !!!!!!!!!!!!! ");
                    
                    agent1.EndEpisode();
                    agent2.EndEpisode();
                }
               
                // Debug.Log( " Episode ends at target position maintained !!!!!!!!!!!!! ");
            //    Debug.Log("Episode ends at the final 3s success target hits! ");
            }
        }
        else
        {
            rendermat.material = crosslose;
            collisiontime = 0f;
        }

        // to avoid ball penetrate into the board per frame
        AvoidPenetrate();
    }

    void AvoidPenetrate()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, Mathf.Infinity))
        {
            Vector3 hit_direction = Vector3.down * hit.distance;
            // Debug.DrawRay(transform.position, hit_direction, Color.black);

            Vector3 project_direction = Vector3.Project(hit_direction, boardrigid.transform.TransformDirection(Vector3.down));
            // Debug.DrawRay(transform.position, project_direction, Color.blue);

            float penetrate_distance = project_direction.magnitude - transform.lossyScale.x/2f;
            if (penetrate_distance < 0)
            {
                Vector3 a = project_direction.normalized * penetrate_distance;
                transform.position = new Vector3(transform.position.x + a.x, transform.position.y + a.y, transform.position.z+a.z);
            } 
        }
    }
    
}



