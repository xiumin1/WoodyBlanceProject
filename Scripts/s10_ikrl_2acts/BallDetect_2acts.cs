
using UnityEngine;

public class BallDetect_2acts : MonoBehaviour
{
    public WoodyArea_ik woodyarea;
    public WoodyAgent_2acts agent1, agent2;
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
        if (Vector3.Distance(woodyarea.ball.position, agent1.target.position) < 0.02f)
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
        }
        else
        {
            rendermat.material = crosslose;
            collisiontime = 0f;
        }
    }

    private void OnCollisionStay(Collision col)
    {
        if (col.gameObject.CompareTag("tray"))
        {
            ballrigid.velocity = new Vector3(ballrigid.velocity.x, boardrigid.velocity.y, ballrigid.velocity.z);
            // ballrigid.angularVelocity = new Vector3(0f,0f,0f);
            
            // this.transform.position = new Vector3(this.transform.position.x, 0.005f + this.transform.position.y, this.transform.position.z);
            // Debug.Log("BALL HIT TRAY called--------------------" + col.relativeVelocity.ToString("F5"));
            
            var a = col.GetContact(0).point;
        
            var b = a - this.transform.position;
            this.transform.position = Vector3.MoveTowards(this.transform.position, b, -1f*Vector3.Distance(ballrigid.velocity,boardrigid.velocity)*Time.deltaTime);
            // this.transform.position = new Vector3(this.transform.position.x, this.transform.position.y + (0.045f-b), this.transform.position.z);
            // var c = Vector3.Distance(a, this.transform.position);
            // Debug.Log("contact point: " + a.ToString("F4") + "  ,distance: " + b + ", " + c);
        }

    }
    // private void OnCollisionEnter(Collision col)
    // {
    //     // var a = col.GetContact(0).point;
        
    //     // var b = Vector3.Distance(a, this.transform.position);
    //     // Debug.Log("contact point: " + a.ToString("F4") + "  ,distance: " + b);
    // }
}



