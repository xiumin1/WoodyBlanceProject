using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;
using System.Reflection;
// command to train on terminal: mlagents-learn config/poca/s17.yaml --env=exe_folder/s17/UnityEnvironment.exe --run-id=s17_3 --train
// command to run torque: mlagents-learn config/poca/s17.yaml --env=exe_folder/s17_1/UnityEnvironment.exe --run-id=s17_5 --no-graphics --resume
// command to run force: mlagents-learn config/poca/s17.yaml --env=exe_folder/s17_act1/UnityEnvironment.exe --run-id=s17_act1_4 --base-port=5010 --num-areas=4 --num-envs=4 --no-graphics --resume
public class EnvController_s17 : MonoBehaviour
{
    [System.Serializable]
    public class AgentInfo
    {
        public WoodyAgent_s17 agent;
        [HideInInspector]
        public Trans2 master;
        [HideInInspector]
        public Trans2 pelvis;
        [HideInInspector]
        public Trans2 leg_right;
        [HideInInspector]
        public Trans2 upper_leg_right;
        [HideInInspector]
        public Trans2 lower_leg_right;
        [HideInInspector]
        public Trans2 foot_right;
        [HideInInspector]
        public Trans2 leg_left;
        [HideInInspector]
        public Trans2 upper_leg_left;
        [HideInInspector]
        public Trans2 lower_leg_left;
        [HideInInspector]
        public Trans2 foot_left;
        [HideInInspector]

        public Trans2 upper_body;
        [HideInInspector]
        public Trans2 arm_right;
        [HideInInspector]
        public Trans2 upper_arm_right;
        [HideInInspector]
        public Trans2 lower_arm_right;
        [HideInInspector]
        public Trans2 hand_right;
        [HideInInspector]
        public Trans2 arm_left;
        [HideInInspector]
        public Trans2 upper_arm_left;
        [HideInInspector]
        public Trans2 lower_arm_left;
        [HideInInspector]
        public Trans2 hand_left;
        [HideInInspector]
        public Trans2 head;

        // for trimatch reward calculation
        [HideInInspector]
        public float pre_l1;
        [HideInInspector]
        public float pre_r1;
        [HideInInspector]
        public float pre_l1rot;
        [HideInInspector]
        public float pre_r1rot;

        [HideInInspector]
        public Vector3 lpre_upper_arm;
        [HideInInspector]
        public Vector3 lpre_lower_arm;
        [HideInInspector]
        public Vector3 lpre_hand;
        [HideInInspector]
        public Vector3 rpre_upper_arm;
        [HideInInspector]
        public Vector3 rpre_lower_arm;
        [HideInInspector]
        public Vector3 rpre_hand;

    }
    /// <summary>
    /// Max Academy steps before this platform resets
    /// </summary>
    /// <returns></returns>
    [Header("Max Environment Steps")] public int MaxEnvironmentSteps = 500;

    //List of Agents On Platform
    public List<AgentInfo> AgentsList = new List<AgentInfo>();
    private PushBlockSettings m_PushBlockSettings;
    private SimpleMultiAgentGroup m_AgentGroup;

    private int m_ResetTimer, stepcounts=0;
    public Transform board, ball, ball_target, ball_target_tmark, path;
    public GameObject support;
    [HideInInspector]
    public Rigidbody ballrb;
    [HideInInspector]
    public Rigidbody boardrb;

    Renderer targetRenderer;

    [HideInInspector]
    public Vector3 pos_agent1, pos_agent2;
    [HideInInspector]
    public Vector3 rotang_agent1, rotang_agent2;
    [HideInInspector]
    public Material crosswin, crosslose;
    [HideInInspector]
    Vector3 pos_board;
    [HideInInspector]
    public float y_target, y_ball; // the y position of the target and ball
    // float y_agent = 0f; // the agent standing position, because the original model has a cylinder to stand
    float z_board_min, z_board_max, x_board_min, x_board_max; // the range of the board in x and z direction
    float rad=0f; 
    float speed=0.25f;
// /////////////define for reward calculation

    [HideInInspector]
    public Vector3 board_target;
    bool fixedboard = true; // to decide if need to randomize the board

    string log = ""; // use to hold the log information for check in standalone exe

    float ball_balance_ratio =0.2f,  board_ratio = 0.3f, trimatch_ratio = 0.25f;
    // for trimatch_test reward ratio, to only consider trimatch ratio
    // float ball_balance_ratio =0,  board_ratio = 0, trimatch_ratio = 1f;
    // float ball_balance_ratio =0.4f,  board_ratio = 0.6f, trimatch_ratio = 0f;

    float pathx = 0, pathz = 0;

    int fixupdate_step=0;

    public int episode_count = 0;
    void Start()
    {
        targetRenderer = ball_target.GetComponent<Renderer>();
        // Starting material
        
        AreaInit();
        // Initialize TeamManager
        m_AgentGroup = new SimpleMultiAgentGroup();
        foreach (var item in AgentsList)
        {
            item.master = new Trans2(item.agent.transform.FindChildObjectwithName("master"));
            item.pelvis = new Trans2(item.agent.transform.FindChildObjectwithName("pelvis"));
            item.leg_right = new Trans2(item.agent.transform.FindChildObjectwithName("leg_right"));
            item.upper_leg_right = new Trans2(item.agent.transform.FindChildObjectwithName("upper_leg_right"));
            item.lower_leg_right = new Trans2(item.agent.transform.FindChildObjectwithName("lower_leg_right"));
            item.foot_right = new Trans2(item.agent.transform.FindChildObjectwithName("foot_right"));
            item.leg_left = new Trans2(item.agent.transform.FindChildObjectwithName("leg_left"));
            item.upper_leg_left = new Trans2(item.agent.transform.FindChildObjectwithName("upper_leg_left"));
            item.lower_leg_left = new Trans2(item.agent.transform.FindChildObjectwithName("lower_leg_left"));
            item.foot_left = new Trans2(item.agent.transform.FindChildObjectwithName("foot_left"));

            item.upper_body = new Trans2(item.agent.transform.FindChildObjectwithName("upper_body"));
            item.arm_right = new Trans2(item.agent.transform.FindChildObjectwithName("arm_right"));
            item.upper_arm_right = new Trans2(item.agent.transform.FindChildObjectwithName("upper_arm_right"));
            item.lower_arm_right = new Trans2(item.agent.transform.FindChildObjectwithName("lower_arm_right"));
            item.hand_right = new Trans2(item.agent.transform.FindChildObjectwithName("hand_right"));
            item.arm_left = new Trans2(item.agent.transform.FindChildObjectwithName("arm_left"));
            item.upper_arm_left = new Trans2(item.agent.transform.FindChildObjectwithName("upper_arm_left"));
            item.lower_arm_left = new Trans2(item.agent.transform.FindChildObjectwithName("lower_arm_left"));
            item.hand_left = new Trans2(item.agent.transform.FindChildObjectwithName("hand_left"));
            item.head = new Trans2(item.agent.transform.FindChildObjectwithName("head"));
            m_AgentGroup.RegisterAgent(item.agent);
        }

        ResetScene();
    }

    void FixedUpdate()
    {
        log = "";
        stepcounts += 1;
        m_ResetTimer += 1;
        if (m_ResetTimer >= MaxEnvironmentSteps && MaxEnvironmentSteps > 0)
        {
            m_AgentGroup.GroupEpisodeInterrupted();
            ResetScene();
        }

        //Hurry Up Penalty
        m_AgentGroup.AddGroupReward(-0.5f / MaxEnvironmentSteps);
        m_AgentGroup.GetRegisteredAgents();

        // to get reward
        foreach(var item in AgentsList)
        {
            DenseRewardTriMatch(item);
            // DenseRewardTriMatch1(item);
            // Debug.Log("action_step: " + item.agent.action_step);
            // Debug.Log("obs_step: " + item.agent.obs_step);
        }     
        DenseRewardBoard();
        DenseRewardBallBalance();

        // ReachGoalReward();
        OutofRangeReward();
        // GetReward();

        rad = FollowPath(rad);

        if (stepcounts > 200)
        {
            support.SetActive(false);
        }
        
        AvoidPenetrate(ball, boardrb); // avoid ball to penetrate into the board

        Debug.Log(log);

        fixupdate_step += 1;

        // Debug.Log("fixupdate_step: " + fixupdate_step);
    }

    /// <summary>
    /// Swap ground material, wait time seconds, then swap back to the regular material.
    /// </summary>
    IEnumerator GoalScoredSwapGroundMaterial(Material mat, float time)
    {
        targetRenderer.material = mat;
        yield return new WaitForSeconds(time); // Wait for 2 sec
        targetRenderer.material = crosslose;
    }

    /// <summary>
    /// Called when the agent moves the block into the goal.
    /// </summary>
    public void TerminateReward(float test)
    {
        //Give Agent Rewards
        m_AgentGroup.SetGroupReward(-1f);

        m_AgentGroup.EndGroupEpisode();
        
        ResetScene();
    }

    public void ReachGoalReward()
    {
        var dist = Vector3.Distance(ball.position, ball_target_tmark.position);
        if(dist<0.02f)
        {
            // set a max step reward for reaching the goal
            foreach(var item in AgentsList)
            {
                item.agent.SetReward(1f);
            }
            m_AgentGroup.SetGroupReward(1f);

            StartCoroutine(GoalScoredSwapGroundMaterial(crosswin, 1f));
            m_AgentGroup.EndGroupEpisode();
            ResetScene();
        }
    }

    public void OutofRangeReward()
    {
        // check if the board go out of the area range
        float distx = Mathf.Abs(board.position.x - this.transform.position.x);
        float distz = Mathf.Abs(board.position.z - this.transform.position.z);
        float disty = Mathf.Abs(board.position.y - this.transform.position.y);

        if (distx>1f || distz > 1f || disty>1.5f || Vector3.Distance(board.position, ball.position)>0.6f)
        {
            
            Debug.Log( " Episode ends at going out of area range! ");
            //Give Agent Rewards
            m_AgentGroup.SetGroupReward(-1f);

            m_AgentGroup.EndGroupEpisode();
            
            ResetScene();   
                
        }
    }
    public void ResetScene()

    {
        
        m_ResetTimer = 0;
        stepcounts = 0;
        support.SetActive(true);

        //Reset Agents
        AgentInitPoseReset();
        //Reset area
        AreaReset(); 

        episode_count += 1;
        foreach (var item in AgentsList)
        {
            item.agent.action_step = 0;
            item.agent.obs_step = 0;

            item.lpre_upper_arm = new Vector3(0f, 0f, 0f);
            item.lpre_lower_arm = new Vector3(0f, 0f, 0f);
            item.lpre_hand = new Vector3(0f, 0f, 0f);

            item.rpre_upper_arm = new Vector3(0f, 0f, 0f);
            item.rpre_lower_arm = new Vector3(0f, 0f, 0f);;
            item.rpre_hand = new Vector3(0f, 0f, 0f);
        }
        // Debug.Log("RESET SCENE------------------");
    }

    public void AreaInit()
    {
        pos_board = board.position; // get the board location
        y_target = ball_target.position.y; // get the height of the target, because it has to be slightly higher than the board to appear
        y_ball = ball.position.y + 0.02f; // get the height of the ball, because it has to be more higher than the board to drop
        board_target = new Vector3(board.position.x, 1.2f, board.position.z);

        // z_board_max = pos_board.z + board.lossyScale.z / 2;
        // z_board_min = pos_board.z - board.lossyScale.z / 2;

        // x_board_max = pos_board.x + board.lossyScale.x / 2;
        // x_board_min = pos_board.x - board.lossyScale.x / 2;
        // // put the two agents at the two sides of the board
        // pos_agent1 = new Vector3(pos_board.x , y_agent, z_board_max + 0.2f);
        // pos_agent2 = new Vector3(pos_board.x , y_agent, z_board_min - 0.2f);

        // // rotate the two agents to face at the board
        // rotang_agent1 = new Vector3(0f, board.eulerAngles.y - 90f, 0f);
        // rotang_agent2 = new Vector3(0f, board.eulerAngles.y + 90f, 0f);
        //rotang_agent2 = Quaternion.Euler(0, board.eulerAngles.y + 90, 0);

        // get the materials

        crosswin = Resources.Load<Material>("Materials/crosswin");
        crosslose = Resources.Load<Material>("Materials/crosslose");

        //woodysource = Resources.Load<GameObject>("Prefabs/woody_inuse6");
        // woodysource = Resources.Load<GameObject>("Prefabs/woody_inuse6_trimatch");

        ballrb = ball.GetComponent<Rigidbody>();
        boardrb = board.GetComponent<Rigidbody>();
        // boardrb.mass = 1f;

    }

    public void AreaReset()
    {
        // init board posotion to its original position
        // board.position = new Vector3(pos_board.x, y_target, pos_board.z);
        if (fixedboard)
        {
            board.position = new Vector3(pos_board.x, y_target, pos_board.z);
            board.eulerAngles = new Vector3(0f,0f,0f);
        }
        else
        {
            float x = Random.Range(pos_board.x-0.1f, pos_board.x+0.1f);
            float z = Random.Range(pos_board.z-0.1f, pos_board.z+0.1f);
            board.position = new Vector3(x, y_target, z);
            float roty = Random.Range(-10.0f, 10.0f);
            board.eulerAngles = new Vector3(0, roty, 0);
            support.transform.position = new Vector3(x, support.transform.position.y, z);
            support.transform.eulerAngles = new Vector3(0, roty, 0f);
        }

        boardrb.velocity = Vector3.zero;
        boardrb.angularVelocity = Vector3.zero;

        ballrb.velocity = Vector3.zero;
        ballrb.angularVelocity = Vector3.zero;
        
        ball_target.position = new Vector3(Random.Range(board.position.x - 0.15f, board.position.x + 0.15f), ball_target.position.y, Random.Range(board.position.z - 0.15f, board.position.z + 0.15f));
        ////target.rotation = Quaternion.identity;
        ball_target.localEulerAngles = new Vector3(0, 0, 0);
        // init the target material to be lose cross, the orange color
        ball_target.GetComponent<Renderer>().material = crosslose;
        // target.GetChild(0).GetComponent<Renderer>().material = crosslose;

        z_board_max = board.position.z + board.lossyScale.z / 2;
        z_board_min = board.position.z - board.lossyScale.z / 2;

        x_board_max = board.position.x + board.lossyScale.x / 2;
        x_board_min = board.position.x - board.lossyScale.x / 2;
        // init ball position by randomizing the ball above the board
        ball.position = new Vector3(Random.Range(x_board_min + ball.lossyScale.x, x_board_max - ball.lossyScale.x), y_ball, Random.Range(z_board_min + ball.lossyScale.z, z_board_max - ball.lossyScale.z));

        while (Vector3.Distance(ball.position, ball_target_tmark.position) < 0.3f)
        {
            ball.position = new Vector3(Random.Range(x_board_min + ball.lossyScale.x, x_board_max - ball.lossyScale.x), y_ball, Random.Range(z_board_min + ball.lossyScale.z, z_board_max - ball.lossyScale.z));
            // Debug.Log("re position ball------ ");
        }
        // init the ball iskenamatic at the begining
        //ball.GetComponent<Rigidbody>().isKinematic = true;
        ballrb.velocity = Vector3.zero;
        ballrb.angularVelocity = Vector3.zero;
        ballrb.Sleep();

        if (support.activeSelf == false)
        {
           support.SetActive(true);
        }

        pathx = Random.Range(0, path.lossyScale.x/2);
        pathz = Random.Range(0, path.lossyScale.z/2);
        rad = Random.Range(0,7);

        // Debug.Log("area reset-------------------------------------------------");
        // for second way of calculating trimatch reward, give pre calculate distance 

        foreach (var item in AgentsList)
        {
            item.pre_l1 = Vector3.Distance(item.agent.l1.position, item.agent.al1.position);
            item.pre_r1 = Vector3.Distance(item.agent.r1.position, item.agent.ar1.position);
            item.pre_l1rot = 1f - Mathf.Pow(Quaternion.Dot(item.agent.l1.rotation, item.agent.al1.rotation), 2f);
            item.pre_r1rot = 1f - Mathf.Pow(Quaternion.Dot(item.agent.r1.rotation, item.agent.ar1.rotation), 2f);
        }
    }

    public void AgentInitPoseReset()
    {
        // var agents = m_AgentGroup.GetRegisteredAgents();
        foreach(var item in AgentsList)
        {
            item.agent.transform.FindChildObjectwithName("master").LoadTrans(item.master);
            item.agent.transform.FindChildObjectwithName("pelvis").LoadTrans(item.pelvis);
            item.agent.transform.FindChildObjectwithName("leg_right").LoadTrans(item.leg_right);
            item.agent.transform.FindChildObjectwithName("upper_leg_right").LoadTrans(item.upper_leg_right);
            item.agent.transform.FindChildObjectwithName("lower_leg_right").LoadTrans(item.lower_leg_right);
            item.agent.transform.FindChildObjectwithName("foot_right").LoadTrans(item.foot_right);
            item.agent.transform.FindChildObjectwithName("leg_left").LoadTrans(item.leg_left);
            item.agent.transform.FindChildObjectwithName("upper_leg_left").LoadTrans(item.upper_leg_left);
            item.agent.transform.FindChildObjectwithName("lower_leg_left").LoadTrans(item.lower_leg_left);
            item.agent.transform.FindChildObjectwithName("foot_left").LoadTrans(item.foot_left);

            item.agent.transform.FindChildObjectwithName("upper_body").LoadTrans(item.upper_body);
            item.agent.transform.FindChildObjectwithName("arm_right").LoadTrans(item.arm_right);
            item.agent.transform.FindChildObjectwithName("upper_arm_right").LoadTrans(item.upper_arm_right);
            item.agent.transform.FindChildObjectwithName("lower_arm_right").LoadTrans(item.lower_arm_right);
            item.agent.transform.FindChildObjectwithName("hand_right").LoadTrans(item.hand_right);
            item.agent.transform.FindChildObjectwithName("arm_left").LoadTrans(item.arm_left);
            item.agent.transform.FindChildObjectwithName("upper_arm_left").LoadTrans(item.upper_arm_left);
            item.agent.transform.FindChildObjectwithName("lower_arm_left").LoadTrans(item.lower_arm_left);
            item.agent.transform.FindChildObjectwithName("hand_left").LoadTrans(item.hand_left);
            item.agent.transform.FindChildObjectwithName("head").LoadTrans(item.head);
        } 
    }

    public float FollowPath(float rad)
    {
        rad += speed * Time.deltaTime;
        float x = path.localPosition.x + pathx * Mathf.Cos(rad);
        float z = path.localPosition.z + pathz * Mathf.Sin(rad);
        float y = path.localPosition.y;

        // target.position = new Vector3(x,y,z);
        ball_target.position = path.TransformPoint(new Vector3(x,y,z));
        return rad;
    }

    public void DenseRewardBallBalance()
    {
        var cur_balldist = Vector3.Distance(ball_target_tmark.position, ball.position) ;
        var reward = 0.0f;
        reward += Mathf.Exp(-cur_balldist*cur_balldist*25f);

        m_AgentGroup.AddGroupReward(reward * ball_balance_ratio);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + ", " + (reward * ball_balance_ratio).ToString("F4") ;//+ ", " + cur_balldist.ToString();
        // Debug.Log(", " + mb.Name + ", " + (reward * ball_balance_ratio).ToString());
    }

    public void DenseRewardBoard()
    {
        var cur_boarddist = Vector3.Distance(board.position, board_target);
        var cur_boardang = 1f - Mathf.Pow(Quaternion.Dot(board.rotation, Quaternion.identity),2f);
        
        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 5f);
        // var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 50f);
        var r_ang = Mathf.Exp( - cur_boardang* 20f);

        reward += r_dist * 0.8f + r_ang * 0.2f; // the ratio change here is to make the reward more sensitive to distance.
        
        m_AgentGroup.AddGroupReward(reward * board_ratio);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString("F4");// + ", distval " + r_dist * 0.8f + ", angleval " + r_ang * 0.2f;
        // log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + cur_boarddist + ", angleval " + cur_boardang;
        // boardreach_reward = reward * board_ratio;
        // Debug.Log(", " + mb.Name + " " + (reward*board_ratio).ToString());
    }

    public void DenseRewardTriMatch(AgentInfo item)
    {
        var cur_l1 = Vector3.Distance(item.agent.l1.position, item.agent.al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(item.agent.l1.rotation, item.agent.al1.rotation),2f);
        var cur_r1 = Vector3.Distance(item.agent.r1.position, item.agent.ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(item.agent.r1.rotation, item.agent.ar1.rotation),2f);

        var r_dist =  Mathf.Exp(- (cur_l1*cur_l1 + cur_r1*cur_r1) * 10f);
        var r_ang = Mathf.Exp(- (cur_l1rot + cur_r1rot));

        var reward = (r_dist * 0.8f + r_ang * 0.2f) * trimatch_ratio;


        // item.agent.AddReward(reward);
        m_AgentGroup.AddGroupReward(reward);
        // Debug.Log(this.name + " dist reward: " + r_dist.ToString("F3") + ", ang reward: " + r_ang.ToString("F3"));
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F4") ;//+ " dist reward: " + r_dist.ToString("F3") + ", ang reward: " + r_ang.ToString("F3");
        // Debug.Log(", " + mb.Name + " " + (reward).ToString("F3") + " dist reward: " + r_dist.ToString("F3") + ", ang reward: " + r_ang.ToString("F3"));

        // handreach_reward = reward;
    }

    public void DenseRewardTriMatch1(AgentInfo item)
    {
        var cur_l1 = Vector3.Distance(item.agent.l1.position, item.agent.al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(item.agent.l1.rotation, item.agent.al1.rotation),2f);
        var cur_r1 = Vector3.Distance(item.agent.r1.position, item.agent.ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(item.agent.r1.rotation, item.agent.ar1.rotation),2f);

        var reward = 0f;

        if(cur_l1 <=0.01f)
        {
            reward += 0.11f;
        }
        else
        {
            if (cur_l1 < item.pre_l1) reward += 0.1f;
            else reward += -0.1f;
        }

        if(cur_r1 <= 0.01f)
        {
            reward += 0.11f;
        }
        else
        {
            if (cur_r1 < item.pre_r1) reward += 0.1f;
            else reward += -0.1f;
        }
        
        if(cur_l1rot <= 0.01f)
        {
            reward += 0.11f;
        }
        else
        {
            if (cur_l1rot < item.pre_l1rot) reward += 0.1f;
            else reward += -0.1f;
        }

        if(cur_r1rot <= 0.01f)
        {
            reward += 0.11f;
        }
        else
        {
            if (cur_r1rot < item.pre_r1rot) reward += 0.1f;
            else reward += -0.1f;
        }

        // if (cur_l1 < item.pre_l1) reward += 0.4f;
        // if (cur_r1 < item.pre_r1) reward += 0.4f;
        // if(cur_l1rot < item.pre_l1rot) reward += 0.1f;
        // if(cur_r1rot < item.pre_r1rot) reward += 0.1f;

        // item.agent.AddReward(reward * trimatch_ratio);
        m_AgentGroup.AddGroupReward(reward * trimatch_ratio);

        item.pre_l1 = cur_l1;
        item.pre_r1 = cur_r1;
        item.pre_l1rot = cur_l1rot;
        item.pre_r1rot = cur_r1rot;

        // Debug.Log("item pre values: " + item.pre_l1);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F4") ;
    }

    void AvoidPenetrate(Transform ball, Rigidbody boardrb)
    {
        RaycastHit hit;
        if (Physics.Raycast(ball.position, Vector3.down, out hit, Mathf.Infinity))
        {
            Vector3 hit_direction = Vector3.down * hit.distance;
            // Debug.DrawRay(transform.position, hit_direction, Color.black);

            Vector3 project_direction = Vector3.Project(hit_direction, boardrb.transform.TransformDirection(Vector3.down));
            // Debug.DrawRay(transform.position, project_direction, Color.blue);
            float penetrate_distance = project_direction.magnitude - ball.lossyScale.x/2f;
            if (penetrate_distance < 0)
            {
                Vector3 a = project_direction.normalized * penetrate_distance;
                ball.position = new Vector3(ball.position.x + a.x, ball.position.y + a.y, ball.position.z+a.z);
            } 
        }
    }
}
