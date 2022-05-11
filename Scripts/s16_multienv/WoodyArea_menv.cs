using UnityEngine;

public class WoodyArea_menv : MonoBehaviour
{
    // public Transform holder_agent1, holder_agent2; // this two boxes use to hold the agent instance, because the agent's physics posture need to be reset to zero after each episode, 
                                      // one of the way could be destroying the current agent instance and then create a new one. 
                                      // Because there is no way to move the agent joint back to its original position without taking enough time.
                 
    // GameObject woodysource, agent1, agent2; // to create the wood agent object
    [HideInInspector]
    public Transform agent1, agent2;
    //[HideInInspector]
    public Transform board, ball, target, ground;

    [HideInInspector]
    public GameObject support;
    Rigidbody ballrb;

    [HideInInspector]
    public Vector3 pos_agent1, pos_agent2;
    [HideInInspector]
    public Vector3 rotang_agent1, rotang_agent2;
    [HideInInspector]
    public Material crosswin, crosslose;
    [HideInInspector]
    public float board_length = 0.0f;

    Vector3 pos_board;
    [HideInInspector]
    public float y_target, y_ball; // the y position of the target and ball
    float y_agent = 0f; // the agent standing position, because the original model has a cylinder to stand
    float z_board_min, z_board_max, x_board_min, x_board_max; // the range of the board in x and z direction
    [HideInInspector]
    public Transform a1L, a1R, a2L, a2R; // the 4 joints on board

    float diagnal_length=0f;

    // a list of tranforms for agent bones information for agent posture resetting
    //agent1
    Trans2 master1;
    Trans2 pelvis1;
    Trans2 leg_right1;
    Trans2 upper_leg_right1;
    Trans2 lower_leg_right1;
    Trans2 foot_right1;
    Trans2 leg_left1;
    Trans2 upper_leg_left1;
    Trans2 lower_leg_left1;
    Trans2 foot_left1;

    Trans2 upper_body1;
    Trans2 arm_right1;
    Trans2 upper_arm_right1;
    Trans2 lower_arm_right1;
    Trans2 hand_right1;
    Trans2 arm_left1;
    Trans2 upper_arm_left1;
    Trans2 lower_arm_left1;
    Trans2 hand_left1;
    Trans2 head1;
    // agent2
    Trans2 master2;
    Trans2 pelvis2;
    Trans2 leg_right2;
    Trans2 upper_leg_right2;
    Trans2 lower_leg_right2;
    Trans2 foot_right2;
    Trans2 leg_left2;
    Trans2 upper_leg_left2;
    Trans2 lower_leg_left2;
    Trans2 foot_left2;

    Trans2 upper_body2;
    Trans2 arm_right2;
    Trans2 upper_arm_right2;
    Trans2 lower_arm_right2;
    Trans2 hand_right2;
    Trans2 arm_left2;
    Trans2 upper_arm_left2;
    Trans2 lower_arm_left2;
    Trans2 hand_left2;
    Trans2 head2;

    //bool areareset = false;
    //float timer = 60.0f;
    Transform path;
    float rad=0f; 
    float speed=0.3f;

    Rigidbody a1_l1, a1_r1, a2_l1, a2_r1;
    [HideInInspector]
    public float multi_env;

    float pathx=0, pathz = 0;
    void Awake()
    {
        
        agent1 = this.transform.Find("Agent1");
        agent2 = this.transform.Find("Agent2");
        board = this.transform.Find("Board");
        ball = this.transform.Find("Ball");
        ground = this.transform.Find("Ground");
        target = board.Find("Target");
        support = this.transform.Find("support").gameObject;
        a1L = board.Find("a1L");
        a1R = board.Find("a1R");
        a2L = board.Find("a2L");
        a2R = board.Find("a2R");
        path = board.Find("path");

        pos_board = board.position; // get the board location
        y_target = target.position.y; // get the height of the target, because it has to be slightly higher than the board to appear
        y_ball = ball.position.y + 0.01f; // get the height of the ball, because it has to be more higher than the board to drop

        board_length = Mathf.Sqrt(2f) * board.lossyScale.x;


        z_board_max = pos_board.z + board.lossyScale.z / 2;
        z_board_min = pos_board.z - board.lossyScale.z / 2;

        x_board_max = pos_board.x + board.lossyScale.x / 2;
        x_board_min = pos_board.x - board.lossyScale.x / 2;
        // put the two agents at the two sides of the board
        pos_agent1 = new Vector3(pos_board.x , y_agent, z_board_max + 0.2f);
        pos_agent2 = new Vector3(pos_board.x , y_agent, z_board_min - 0.2f);

        // rotate the two agents to face at the board
        rotang_agent1 = new Vector3(0f, board.eulerAngles.y - 90f, 0f);
        rotang_agent2 = new Vector3(0f, board.eulerAngles.y + 90f, 0f);
        //rotang_agent2 = Quaternion.Euler(0, board.eulerAngles.y + 90, 0);

        // get the materials

        crosswin = Resources.Load<Material>("Materials/crosswin");
        crosslose = Resources.Load<Material>("Materials/crosslose");

        //woodysource = Resources.Load<GameObject>("Prefabs/woody_inuse6");
        // woodysource = Resources.Load<GameObject>("Prefabs/woody_inuse6_trimatch");

        diagnal_length = Mathf.Sqrt(2) * board.lossyScale.z;

        ballrb = ball.GetComponent<Rigidbody>();

        if (!a1L.Find("l1").GetComponent<FixedJoint>())
            a1_l1 = a1L.Find("l1").gameObject.AddComponent<FixedJoint>().GetComponent<Rigidbody>();
        else
            a1_l1 = a1L.Find("l1").GetComponent<Rigidbody>();

        if(!a1R.Find("r1").GetComponent<FixedJoint>())
            a1_r1 = a1R.Find("r1").gameObject.AddComponent<FixedJoint>().GetComponent<Rigidbody>();
        else
            a1_r1 = a1R.Find("r1").GetComponent<Rigidbody>();
        
        if(!a2L.Find("l1").GetComponent<FixedJoint>())
            a2_l1 = a2L.Find("l1").gameObject.AddComponent<FixedJoint>().GetComponent<Rigidbody>();
        else
            a2_l1 = a2L.Find("l1").GetComponent<Rigidbody>();

        if(!a2R.Find("r1").GetComponent<FixedJoint>())
            a2_r1 = a2R.Find("r1").gameObject.AddComponent<FixedJoint>().GetComponent<Rigidbody>();
        else
            a2_r1 = a2R.Find("r1").GetComponent<Rigidbody>();
            
        if(!ball.GetComponent<BallDetect_menv>())
            ball.gameObject.AddComponent<BallDetect_menv>();
        if(!ground.GetComponent<GroundDetect_menv>())
            ground.gameObject.AddComponent<GroundDetect_menv>();
        if(!board.GetComponent<BoardCollision_menv>())
            board.gameObject.AddComponent<BoardCollision_menv>();
            
        // Debug.Log("area awake--------------");
    }

    public void AreaReset()
    {
        // init the environment to have a initial distribution
        // init board posotion to its original position
        // board.position = new Vector3(pos_board.x, y_target, pos_board.z);
        float x, z;

        // x = Random.Range(pos_board.x-0.1f, pos_board.x+0.1f);
        // z = Random.Range(pos_board.z-0.1f, pos_board.z+0.1f);
        x = pos_board.x;
        z = pos_board.z;

        board.position = new Vector3(x, y_target, z);

        // var a1_l1_angle = Vector3.Angle(new Vector3(a1_l1.position.x-board.position.x, 0, a1_l1.position.z - board.position.z), board.forward);
        // var a1_r1_angle = Vector3.Angle(new Vector3(a1_r1.position.x-board.position.x, 0, a1_r1.position.z - board.position.z), board.forward);

        // var a2_l1_angle = Vector3.Angle(new Vector3(a2_l1.position.x-board.position.x, 0, a2_l1.position.z - board.position.z), board.forward);
        // var a2_r1_angle = Vector3.Angle(new Vector3(a2_r1.position.x-board.position.x, 0, a2_r1.position.z - board.position.z), board.forward);

        // Debug.Log("a1 l1 pos= " + (a1_l1.position-board.position).ToString("F3") + "a1 r1 pos= " + (a1_r1.position-board.position).ToString("F3") + board.forward);

        // Debug.Log("a1 left= " + a1_l1_angle.ToString("F3") + ", a1 right= " + a1_r1_angle.ToString("F3") + ", a2 left= " + a2_l1_angle + ", a2 right= " + a2_r1_angle);

        float roty = Random.Range(-10.0f, 10.0f);
        board.eulerAngles = new Vector3(0, roty, 0);
        
        // to make sure the init board velocity is 0
        board.GetComponent<Rigidbody>().velocity = Vector3.zero;
        board.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        board.GetComponent<Rigidbody>().Sleep();

        a1_l1.velocity = Vector3.zero;
        a1_l1.angularVelocity = Vector3.zero;
        a1_r1.velocity = Vector3.zero;
        a1_r1.angularVelocity = Vector3.zero;

        a2_l1.velocity = Vector3.zero;
        a2_l1.angularVelocity = Vector3.zero;
        a2_r1.velocity = Vector3.zero;
        a2_r1.angularVelocity = Vector3.zero;

        
        a1_l1.Sleep();
        a1_r1.Sleep();
        a2_l1.Sleep();
        a2_r1.Sleep();

        support.transform.position = new Vector3(x, support.transform.position.y, z);
        support.transform.eulerAngles = new Vector3(0, roty, 0f);

        target.position = new Vector3(Random.Range(board.position.x - 0.15f, board.position.x + 0.15f), target.position.y, Random.Range(board.position.z - 0.15f, board.position.z + 0.15f));
        ////target.rotation = Quaternion.identity;
        target.localEulerAngles = new Vector3(0, 0, 0);
        // init the target material to be lose cross, the orange color
        target.GetComponent<Renderer>().material = crosslose;
        // target.GetChild(0).GetComponent<Renderer>().material = crosslose;

        // z_board_max = board.localPosition.z + board.lossyScale.z / 2;
        // z_board_min = board.localPosition.z - board.lossyScale.z / 2;

        // x_board_max = board.localPosition.x + board.lossyScale.x / 2;
        // x_board_min = board.localPosition.x - board.lossyScale.x / 2;

        z_board_max = board.position.z + board.lossyScale.z / 2;
        z_board_min = board.position.z - board.lossyScale.z / 2;

        x_board_max = board.position.x + board.lossyScale.x / 2;
        x_board_min = board.position.x - board.lossyScale.x / 2;
        // init ball position by randomizing the ball above the board
        ball.position = new Vector3(Random.Range(x_board_min + ball.lossyScale.x, x_board_max - ball.lossyScale.x), y_ball, Random.Range(z_board_min + ball.lossyScale.z, z_board_max - ball.lossyScale.z));

        while (Vector3.Distance(ball.position, target.position) < 0.1f)
        {
            ball.position = new Vector3(Random.Range(x_board_min + ball.lossyScale.x, x_board_max - ball.lossyScale.x), y_ball, Random.Range(z_board_min + ball.lossyScale.z, z_board_max - ball.lossyScale.z));
        }
        // init the ball iskenamatic at the begining
        //ball.GetComponent<Rigidbody>().isKinematic = true;
        ballrb.velocity = new Vector3(0f,0f,0f);
        ballrb.angularVelocity = Vector3.zero;
        ballrb.Sleep();

        if (support.activeSelf == false)
        {
           support.SetActive(true);
        }

        pathx = Random.Range(0, path.lossyScale.x/2);
        pathz = Random.Range(0, path.lossyScale.z/2);
        // Debug.Log("area reset-------------------------------------------------");
    }

    public void AgentInitPoseGet(string agentname)
    {
        if (agentname == "Agent1")
        {
            // agent1 = Instantiate(woodysource, new Vector3(pos_agent1.x, pos_agent1.y, pos_agent1.z), Quaternion.Euler(rotang_agent1.x, rotang_agent1.y, rotang_agent1.z));
            // agent1.transform.parent = holder_agent1;

            master1 = new Trans2(agent1.transform.FindChildObjectwithName("master"));
            pelvis1 = new Trans2(agent1.transform.FindChildObjectwithName("pelvis"));
            leg_right1 = new Trans2(agent1.transform.FindChildObjectwithName("leg_right"));
            upper_leg_right1 = new Trans2(agent1.transform.FindChildObjectwithName("upper_leg_right"));
            lower_leg_right1 = new Trans2(agent1.transform.FindChildObjectwithName("lower_leg_right"));
            foot_right1 = new Trans2(agent1.transform.FindChildObjectwithName("foot_right"));
            leg_left1 = new Trans2(agent1.transform.FindChildObjectwithName("leg_left"));
            upper_leg_left1 = new Trans2(agent1.transform.FindChildObjectwithName("upper_leg_left"));
            lower_leg_left1 = new Trans2(agent1.transform.FindChildObjectwithName("lower_leg_left"));
            foot_left1 = new Trans2(agent1.transform.FindChildObjectwithName("foot_left"));

            upper_body1 = new Trans2(agent1.transform.FindChildObjectwithName("upper_body"));
            arm_right1 = new Trans2(agent1.transform.FindChildObjectwithName("arm_right"));
            upper_arm_right1 = new Trans2(agent1.transform.FindChildObjectwithName("upper_arm_right"));
            lower_arm_right1 = new Trans2(agent1.transform.FindChildObjectwithName("lower_arm_right"));
            hand_right1 = new Trans2(agent1.transform.FindChildObjectwithName("hand_right"));
            arm_left1 = new Trans2(agent1.transform.FindChildObjectwithName("arm_left"));
            upper_arm_left1 = new Trans2(agent1.transform.FindChildObjectwithName("upper_arm_left"));
            lower_arm_left1 = new Trans2(agent1.transform.FindChildObjectwithName("lower_arm_left"));
            hand_left1 = new Trans2(agent1.transform.FindChildObjectwithName("hand_left"));
            head1 = new Trans2(agent1.transform.FindChildObjectwithName("head"));
        }

        // agent2
        if (agentname == "Agent2")
        {
            // agent2 = Instantiate(woodysource, new Vector3(pos_agent2.x, pos_agent2.y, pos_agent2.z), Quaternion.Euler(rotang_agent2.x, rotang_agent2.y, rotang_agent2.z));
            // agent2.transform.parent = holder_agent2;
            //agent2.transform.Find("body").GetComponent<SkinnedMeshRenderer>().enabled = false;

            master2 = new Trans2(agent2.transform.FindChildObjectwithName("master"));
            pelvis2 = new Trans2(agent2.transform.FindChildObjectwithName("pelvis"));
            leg_right2 = new Trans2(agent2.transform.FindChildObjectwithName("leg_right"));
            upper_leg_right2 = new Trans2(agent2.transform.FindChildObjectwithName("upper_leg_right"));
            lower_leg_right2 = new Trans2(agent2.transform.FindChildObjectwithName("lower_leg_right"));
            foot_right2 = new Trans2(agent2.transform.FindChildObjectwithName("foot_right"));
            leg_left2 = new Trans2(agent2.transform.FindChildObjectwithName("leg_left"));
            upper_leg_left2 = new Trans2(agent2.transform.FindChildObjectwithName("upper_leg_left"));
            lower_leg_left2 = new Trans2(agent2.transform.FindChildObjectwithName("lower_leg_left"));
            foot_left2 = new Trans2(agent2.transform.FindChildObjectwithName("foot_left"));

            upper_body2 = new Trans2(agent2.transform.FindChildObjectwithName("upper_body"));
            arm_right2 = new Trans2(agent2.transform.FindChildObjectwithName("arm_right"));
            upper_arm_right2 = new Trans2(agent2.transform.FindChildObjectwithName("upper_arm_right"));
            lower_arm_right2 = new Trans2(agent2.transform.FindChildObjectwithName("lower_arm_right"));
            hand_right2 = new Trans2(agent2.transform.FindChildObjectwithName("hand_right"));
            arm_left2 = new Trans2(agent2.transform.FindChildObjectwithName("arm_left"));
            upper_arm_left2 = new Trans2(agent2.transform.FindChildObjectwithName("upper_arm_left"));
            lower_arm_left2 = new Trans2(agent2.transform.FindChildObjectwithName("lower_arm_left"));
            hand_left2 = new Trans2(agent2.transform.FindChildObjectwithName("hand_left"));
            head2 = new Trans2(agent2.transform.FindChildObjectwithName("head"));
        }
    }

    public void AgentInitPoseReset(string agentname)
    {
        if (agentname == "Agent1") 
        {
            agent1.transform.FindChildObjectwithName("master").LoadTrans(master1);
            agent1.transform.FindChildObjectwithName("pelvis").LoadTrans(pelvis1);
            agent1.transform.FindChildObjectwithName("leg_right").LoadTrans(leg_right1);
            agent1.transform.FindChildObjectwithName("upper_leg_right").LoadTrans(upper_leg_right1);
            agent1.transform.FindChildObjectwithName("lower_leg_right").LoadTrans(lower_leg_right1);
            agent1.transform.FindChildObjectwithName("foot_right").LoadTrans(foot_right1);
            agent1.transform.FindChildObjectwithName("leg_left").LoadTrans(leg_left1);
            agent1.transform.FindChildObjectwithName("upper_leg_left").LoadTrans(upper_leg_left1);
            agent1.transform.FindChildObjectwithName("lower_leg_left").LoadTrans(lower_leg_left1);
            agent1.transform.FindChildObjectwithName("foot_left").LoadTrans(foot_left1);

            agent1.transform.FindChildObjectwithName("upper_body").LoadTrans(upper_body1);
            agent1.transform.FindChildObjectwithName("arm_right").LoadTrans(arm_right1);
            agent1.transform.FindChildObjectwithName("upper_arm_right").LoadTrans(upper_arm_right1);
            agent1.transform.FindChildObjectwithName("lower_arm_right").LoadTrans(lower_arm_right1);
            agent1.transform.FindChildObjectwithName("hand_right").LoadTrans(hand_right1);
            agent1.transform.FindChildObjectwithName("arm_left").LoadTrans(arm_left1);
            agent1.transform.FindChildObjectwithName("upper_arm_left").LoadTrans(upper_arm_left1);
            agent1.transform.FindChildObjectwithName("lower_arm_left").LoadTrans(lower_arm_left1);
            agent1.transform.FindChildObjectwithName("hand_left").LoadTrans(hand_left1);
            agent1.transform.FindChildObjectwithName("head").LoadTrans(head1);
        }

        if (agentname == "Agent2")
        {
            agent2.transform.FindChildObjectwithName("master").LoadTrans(master2);
            agent2.transform.FindChildObjectwithName("pelvis").LoadTrans(pelvis2);
            agent2.transform.FindChildObjectwithName("leg_right").LoadTrans(leg_right2);
            agent2.transform.FindChildObjectwithName("upper_leg_right").LoadTrans(upper_leg_right2);
            agent2.transform.FindChildObjectwithName("lower_leg_right").LoadTrans(lower_leg_right2);
            agent2.transform.FindChildObjectwithName("foot_right").LoadTrans(foot_right2);
            agent2.transform.FindChildObjectwithName("leg_left").LoadTrans(leg_left2);
            agent2.transform.FindChildObjectwithName("upper_leg_left").LoadTrans(upper_leg_left2);
            agent2.transform.FindChildObjectwithName("lower_leg_left").LoadTrans(lower_leg_left2);
            agent2.transform.FindChildObjectwithName("foot_left").LoadTrans(foot_left2);

            agent2.transform.FindChildObjectwithName("upper_body").LoadTrans(upper_body2);
            agent2.transform.FindChildObjectwithName("arm_right").LoadTrans(arm_right2);
            agent2.transform.FindChildObjectwithName("upper_arm_right").LoadTrans(upper_arm_right2);
            agent2.transform.FindChildObjectwithName("lower_arm_right").LoadTrans(lower_arm_right2);
            agent2.transform.FindChildObjectwithName("hand_right").LoadTrans(hand_right2);
            agent2.transform.FindChildObjectwithName("arm_left").LoadTrans(arm_left2);
            agent2.transform.FindChildObjectwithName("upper_arm_left").LoadTrans(upper_arm_left2);
            agent2.transform.FindChildObjectwithName("lower_arm_left").LoadTrans(lower_arm_left2);
            agent2.transform.FindChildObjectwithName("hand_left").LoadTrans(hand_left2);
            agent2.transform.FindChildObjectwithName("head").LoadTrans(head2);
        }
    }

    public float FollowPath(float rad)
    {
        rad += speed*Time.deltaTime;
        float x = path.localPosition.x + pathx * Mathf.Cos(rad);
        float z = path.localPosition.z + pathz * Mathf.Sin(rad);
        float y = path.localPosition.y;

        // target.position = new Vector3(x,y,z);
        target.position = path.TransformPoint(new Vector3(x,y,z));
        return rad;
    }

    void FixedUpdate()
    {
        rad = FollowPath(rad);
    }
}

