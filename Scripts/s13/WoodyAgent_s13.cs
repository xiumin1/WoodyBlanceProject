using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

using System.Collections;
using System.Reflection;
public class WoodyAgent_s13 : Agent
{
    //------------------------------------------------------------------------------------------
    // the two agents init position and the whole environment info will be initilized and reset in the woodyarea script for simplicity
    public WoodyAgent_s13 otheragent;
    WoodyArea_ik woodyarea;
    Transform board, ball;
    public Transform target;
    GameObject support;
    Rigidbody boardrigid, ballrigid;
    Vector3 leftforce, rightforce;
    Rigidbody upper_arm_right, lower_arm_right, hand_right, upper_arm_left, lower_arm_left, hand_left;
    Transform hand_right_end, hand_left_end;
    Transform upper_arm_right_joint, lower_arm_right_joint, hand_right_joint, upper_arm_left_joint, lower_arm_left_joint, hand_left_joint;

    int action_count = 0;
    Material crosswin, crosslose;
    public Transform l1, l2, l3, r1, r2, r3; // the three holding points on the board
    Transform al1, al2, al3, ar1, ar2, ar3;// the three holding points on the agent hands
    // use to calculate rewards
    float cur_l1, cur_l2, cur_l3, cur_r1, cur_r2, cur_r3; // the current distance of holding point between hand and board
    float pre_l1, pre_l2, pre_l3, pre_r1, pre_r2, pre_r3;

    float pre_hand_dist, pre_hand_ang;
    float pre_balldist, cur_balldist; // to award the reward when the ball arrives to the target location
    float pre_boarddist, cur_boarddist, pre_boardang, cur_boardang; // use to calculate reward for board balance in position and angle
    Vector3 board_target; // the target position where the board supposed to stay at balance status
    // use to access parameters from outside of C# and Unity
    EnvironmentParameters m_ResetParams;
    float force_magnitude = 0.0f;  // the force magnitude adds to each joint
    float terminate_steps = 0;     // the max steps needs for each episode, use to calculate the time penalty

    float target_distance = 0.05f; // the target distance bewtween hand to holding location, 
    float target_angle = 0.05f;       // the target angle distance between hand to up direction

    float remove_support_episodes = 0f; // to remove the support from the environment after training this much episodes
    [HideInInspector]
    public float target_maintain_time = 0f;

    [HideInInspector]
    public float cam_rot_speed = 0.0f;
    [HideInInspector]
    public float cam_look_distance = 1.5f;

    Vector3 upper_arm_right_torque, lower_arm_right_torque, hand_right_torque, upper_arm_left_torque, lower_arm_left_torque, hand_left_torque;

    float ball_balance_ratio = 1f;
    float board_ratio = 1f;
    float comfort_pose_ratio = 1f;
    float trimatch_ratio = 1f;
    float observe_index = 0f;
    float action_index = 0f;
    float use_support = 0f;
    float comfort_dense_reward_index = 0f;
    float comfort_sparse_reward_index = 0f;
    float board_dense_reward_index = 0f;
    float board_sparse_reward_index = 0f;
    float ball_balance_dense_reward_index = 0f;
    float ball_balance_sparse_reward_index = 0f;
    float trimatch_dense_reward_index = 0f;
    float trimatch_sparse_reward_index = 0f;
    float print_pythonparams = 0f;
    float print_rewardlog = 0f;
    float r_reach = 0.02f;
    float r_approch = 0.01f; //check scene5
    float time_penalty_reward = 0f;
    string log = ""; // use to hold the log information for check in standalone exe

    Vector3 jn_upper_arm_right, jn_lower_arm_right, jn_hand_right, jn_upper_arm_left, jn_lower_arm_left, jn_hand_left;
    int stepcounts = 0;
    int episodecounts = 0;
    // float time = 0f;
    float handreach_reward = 0f; // the following three rewards is to adding them into the observation as extra 3 dimensions, to pass them into the python API for CRPO approach
    float boardreach_reward = 0f;
    float ballreach_reward = 0f;
    public override void Initialize()
    {
        woodyarea = GameObject.Find("WoodyArea").GetComponent<WoodyArea_ik>();
        woodyarea.AreaInit();

        board = woodyarea.board;
        // target = woodyarea.target;
        ball = woodyarea.ball;
        support = woodyarea.support;

        boardrigid = board.gameObject.GetComponent<Rigidbody>();
        ballrigid = ball.gameObject.GetComponent<Rigidbody>();

        board_target = new Vector3(board.position.x, board.position.y + 0.1f, board.position.z);

        crosswin = woodyarea.crosswin;
        crosslose = woodyarea.crosslose;

        // each agent script will only need to focus on one agent's partial obersvation and action
        GetNeutralPos(this.name); // to get the neutral position of both arms for comfortable pose 
        //// init agent's instance, its location and rotation, and all jionts rigidbody
        woodyarea.AgentInitPoseGet(this.name);

        // get the reference of all needed joints
        GetJointsRef();
        //agentfacedir = agent.transform.forward;

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        // SetResetParameters();
        // initialize the pre values for distance or angle calculation for reward
        al1 = FindChildObjectwithName(this.transform, "l1");
        al2 = FindChildObjectwithName(this.transform, "l2");
        al3 = FindChildObjectwithName(this.transform, "l3");

        ar1 = FindChildObjectwithName(this.transform, "r1");
        ar2 = FindChildObjectwithName(this.transform, "r2");
        ar3 = FindChildObjectwithName(this.transform, "r3");

        // SetResetParameters();
        // if (observe_index== 3 || observe_index==4)
        // {
        //     this.transform.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>().BrainParameters.VectorObservationSize = 126;
        // }
        
        // if (observe_index== 1 || observe_index==2)
        // {
        //     this.transform.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>().BrainParameters.VectorObservationSize = 123;
        // }

        action_count = this.transform.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>().BrainParameters.ActionSpec.NumContinuousActions;
    }

    // to get save the neutral position at the begining of the agent character, so that it could be used to calcualte the comfort pose reward
    void GetNeutralPos(string agentname)
    {
        if (this.name == "Agent1")
        {
            jn_upper_arm_right = new Vector3(-0.1f, 1.4f, 0.5f);
            jn_lower_arm_right = new Vector3(-0.1f, 1.1f, 0.5f);
            jn_hand_right = new Vector3(-0.1f, 0.8f, 0.5f);

            jn_upper_arm_left = new Vector3(0.1f, 1.4f, 0.5f);
            jn_lower_arm_left = new Vector3(0.1f, 1.1f, 0.5f);
            jn_hand_left = new Vector3(0.1f, 0.8f, 0.5f);
        }

        if (this.name == "Agent2")
        {
            jn_upper_arm_right = new Vector3(0.1f, 1.4f, -0.5f);
            jn_lower_arm_right = new Vector3(0.1f, 1.1f, -0.5f);
            jn_hand_right = new Vector3(0.1f, 0.8f, -0.5f);

            jn_upper_arm_left = new Vector3(-0.1f, 1.4f, -0.5f);
            jn_lower_arm_left = new Vector3(-0.1f, 1.1f, -0.5f);
            jn_hand_left = new Vector3(-0.1f, 0.8f, -0.5f);
        }
    }

    // get the joint reference of the character for the later on calculation
    public void GetJointsRef()
    {
        upper_arm_right = FindChildObjectwithName(this.transform, "upper_arm_right").GetComponent<Rigidbody>();
        lower_arm_right = FindChildObjectwithName(this.transform, "lower_arm_right").GetComponent<Rigidbody>();
        hand_right = FindChildObjectwithName(this.transform, "hand_right").GetComponent<Rigidbody>();
        hand_right_end = FindChildObjectwithName(this.transform, "hand_right_end");
        upper_arm_left = FindChildObjectwithName(this.transform, "upper_arm_left").GetComponent<Rigidbody>();
        lower_arm_left = FindChildObjectwithName(this.transform, "lower_arm_left").GetComponent<Rigidbody>();
        hand_left = FindChildObjectwithName(this.transform, "hand_left").GetComponent<Rigidbody>();
        hand_left_end = FindChildObjectwithName(this.transform, "hand_left_end");

        upper_arm_right_joint = FindChildObjectwithName(this.transform, "upper_arm_right_joint");
        lower_arm_right_joint = FindChildObjectwithName(this.transform, "lower_arm_right_joint");
        hand_right_joint = FindChildObjectwithName(this.transform, "hand_right_joint");
        upper_arm_left_joint = FindChildObjectwithName(this.transform, "upper_arm_left_joint");
        lower_arm_left_joint = FindChildObjectwithName(this.transform, "lower_arm_left_joint");
        hand_left_joint = FindChildObjectwithName(this.transform, "hand_left_joint");
    }

    public override void CollectObservations(VectorSensor sensor) // 123 + 3 = 126
    {
        var methodName = "Observation" + observe_index.ToString();

        if(observe_index > 0) SendMessage(methodName, sensor);

        else Debug.Log("There is no observation given!");
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        stepcounts += 1;
        // Debug.Log("step counts: " + stepcounts);
        // Debug.Log(this.name + "ball speed: " + ballrigid.velocity + ", board speed: " + boardrigid.velocity);
        ActionSegment<float> vectoraction = actionBuffers.ContinuousActions;
        
        var methodName = "TakeAction" + action_index.ToString();

        if(action_index > 0) SendMessage(methodName, vectoraction);

        else Debug.Log("There is no action received!");

        log = "";

        ////add reward for lower energy of the arms
        if (comfort_pose_ratio > 0f) GetRewardComfortPose();

        if (board_ratio > 0f) GetRewardBoard();

        if (ball_balance_ratio > 0f) GetRewardBallBalance();
         
        if (trimatch_ratio > 0f) GetRewardTriMatch();

        AddReward(time_penalty_reward);

        if (print_pythonparams == 1f) PrintPythonParams();

        if (print_rewardlog == 1f) Debug.Log(this.name +"----" +  log );

        // Debug.DrawLine(l1.position, al1.position, Color.red);
        // Debug.DrawLine(l2.position, al2.position, Color.green);
        // Debug.DrawLine(l3.position, al3.position, Color.yellow);

        // Debug.DrawLine(r1.position, ar1.position, Color.red);
        // Debug.DrawLine(r2.position, ar2.position, Color.green);
        // Debug.DrawLine(r3.position, ar3.position, Color.yellow);

        // to remove support after training certain episodes, like 3000 episodes
        if (use_support == 0 && stepcounts > 100)
        {
            support.SetActive(false);
        }

        // Debug.Log(" ball mass= " + ball.GetComponent<Rigidbody>().mass + ", board mass= " + board.GetComponent<Rigidbody>().mass);
    }

    void TakeAction1(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*100f;
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }
        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1], vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4], vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7], vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10], vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13], vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16], vectoraction[17]);

        upper_arm_right.AddRelativeForce(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeForce(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeForce(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeForce(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeForce(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeForce(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }

    void TakeAction2(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*100;
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }
        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1], vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4], vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7], vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10], vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13], vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16], vectoraction[17]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }

    void TakeAction3(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*100f;
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }
        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1]/100f, vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4]/100f, vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7]/100f, vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10]/100f, vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13]/100f, vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16]/100f, vectoraction[17]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }

    void TakeAction4(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude*100f;
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }
        upper_arm_right_torque = new Vector3(vectoraction[0], 0, vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], 0, vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], 0, vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], 0, vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], 0, vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], 0, vectoraction[17]);

        upper_arm_right.AddForceAtPosition(upper_arm_right_torque, lower_arm_right_joint.position, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddForceAtPosition(lower_arm_right_torque, hand_right_joint.position, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddForceAtPosition(hand_right_torque, r1.position, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddForceAtPosition(upper_arm_left_torque, lower_arm_left_joint.position, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddForceAtPosition(lower_arm_left_torque, hand_left_joint.position, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddForceAtPosition(hand_left_torque, l1.position, ForceMode.Force); //, limbends["hand_left_end"].position);

        upper_arm_right.AddRelativeTorque(new Vector3(0, vectoraction[1]/100f, 0),ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(new Vector3(0, vectoraction[4]/100f, 0), ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(new Vector3(0, vectoraction[7]/100f, 0), ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(new Vector3(0, vectoraction[10]/100f, 0), ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(new Vector3(0, vectoraction[13]/100f, 0), ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(new Vector3(0, vectoraction[16]/100f, 0), ForceMode.Force); //, limbends["hand_left_end"].position);

    }

    void TakeAction7(ActionSegment<float> vectoraction)
    {
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude * 50f;
            
            // Debug.Log(i + "th action is: " + vectoraction[i]);
        }

        Debug.Log( " TakeAction0 " + this.name + ", " + stepcounts + " action is: " + vectoraction[0].ToString("F3") + ", " + vectoraction[1].ToString("F3") + 
            ", " + vectoraction[2].ToString("F3") + ", " + vectoraction[3].ToString("F3") + ", " + vectoraction[4].ToString("F3") + ", " + 
            vectoraction[5].ToString("F3") + ", " + vectoraction[6].ToString("F3") + ", " + vectoraction[7].ToString("F3") + ", " + vectoraction[8].ToString("F3") +
            vectoraction[9].ToString("F3") + ", " + vectoraction[10].ToString("F3") + ", " + vectoraction[11].ToString("F3"));

        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1]/10f, vectoraction[2]);
        lower_arm_right_torque = new Vector3(0f, vectoraction[3]/10f, vectoraction[4]);
        hand_right_torque = new Vector3(vectoraction[5], 0f, vectoraction[6]);

        upper_arm_left_torque = new Vector3(vectoraction[7], vectoraction[8]/10f, vectoraction[9]);
        lower_arm_left_torque = new Vector3(0f, vectoraction[10]/10f, vectoraction[11]);
        hand_left_torque = new Vector3(vectoraction[12], 0f, vectoraction[13]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);
    }
    public void GetRewardTriMatch()
    {
        var methodName = "";
        if(trimatch_dense_reward_index > 0)
        {
            methodName = "DenseRewardTriMatch" + trimatch_dense_reward_index.ToString();
            SendMessage(methodName);
        }
        if(trimatch_sparse_reward_index > 0)
        {
            methodName = "SparseRewardTriMatch" + trimatch_sparse_reward_index.ToString();
            SendMessage(methodName);
        }  
    }

    public void GetRewardBallBalance()
    {
        var methodName = "";
        if(ball_balance_dense_reward_index > 0)
        {
            methodName = "DenseRewardBallBalance" + ball_balance_dense_reward_index.ToString();
            SendMessage(methodName);
        }
        if(ball_balance_sparse_reward_index > 0)
        {
            methodName = "SparseRewardBallBalance" + ball_balance_sparse_reward_index.ToString();
            SendMessage(methodName);
        }  
    }

    public void GetRewardBoard()
    {
        var methodName = "";
        if(board_dense_reward_index > 0)
        {
            methodName = "DenseRewardBoard" + board_dense_reward_index.ToString();
            SendMessage(methodName);
        }
        if(board_sparse_reward_index > 0)
        {
            methodName = "SparseRewardBoard" + board_sparse_reward_index.ToString();
            SendMessage(methodName);
        }  
    }

    public void GetRewardComfortPose()
    {
        var methodName = "";
        if(comfort_dense_reward_index > 0)
        {
            methodName = "DenseRewardComfortPose" + comfort_dense_reward_index.ToString();
            SendMessage(methodName);
        }
        if(comfort_sparse_reward_index > 0)
        {
            methodName = "SparseRewardComfortPose" + comfort_sparse_reward_index.ToString();
            SendMessage(methodName);
        }
    }

    public override void OnEpisodeBegin()
    {
        woodyarea.AreaReset();

        support.SetActive(true);
        
        // to make sure at the begining of each episode the agent body parts will go back to the original position and orientation.
        woodyarea.AgentInitPoseReset(this.name);
        //reset the force ratio here as a accessible paramters in python api
        SetResetParameters();
        InitRewardCalculator();
        
        stepcounts = 0;

        episodecounts += 1;
        if (episodecounts > remove_support_episodes && remove_support_episodes !=0) use_support =0;
        // time = 0f;

        handreach_reward = 0f;
        boardreach_reward = 0f;
        ballreach_reward = 0f;
    }

    void SetResetParameters()
    {
        // general parameters
        force_magnitude = m_ResetParams.GetWithDefault("force_magnitude", 0.1f);
        terminate_steps = m_ResetParams.GetWithDefault("terminate_steps", 500f);
        remove_support_episodes = m_ResetParams.GetWithDefault("remove_support_episodes", 5f);

        // target distance and anlge value, could also be used for curriculum learning purpose
        target_distance = m_ResetParams.GetWithDefault("target_distance", 0.05f);
        target_angle = m_ResetParams.GetWithDefault("target_angle", 0.05f);
        target_maintain_time = m_ResetParams.GetWithDefault("target_maintain_time", 3f);

        time_penalty_reward = m_ResetParams.GetWithDefault("time_penalty_reward", -0.01f);

        // reward ratio parameters for controlling the weight ratio of each reward 
        ball_balance_ratio = m_ResetParams.GetWithDefault("ball_balance_ratio", 0.024f);
        board_ratio = m_ResetParams.GetWithDefault("board_ratio", 0.136f);
        comfort_pose_ratio = m_ResetParams.GetWithDefault("comfort_pose_ratio", 0.1f);
        trimatch_ratio = m_ResetParams.GetWithDefault("trimatch_ratio", 1f);

        // for camera
        cam_rot_speed = m_ResetParams.GetWithDefault("cam_rot_speed", 0.1f);
        cam_look_distance = m_ResetParams.GetWithDefault("cam_look_distance", 2.1f);

        // to use a flag index to decide which part of code to use, separate sparse reward and dense reward
        observe_index = m_ResetParams.GetWithDefault("observe_index", 4f);
        action_index = m_ResetParams.GetWithDefault("action_index", 1f);
        use_support = m_ResetParams.GetWithDefault("use_support", 1f);

        comfort_dense_reward_index = m_ResetParams.GetWithDefault("comfort_dense_reward_index", 0f);
        comfort_sparse_reward_index = m_ResetParams.GetWithDefault("comfort_sparse_reward_index", 0f);

        board_dense_reward_index = m_ResetParams.GetWithDefault("board_dense_reward_index", 0f);
        board_sparse_reward_index = m_ResetParams.GetWithDefault("board_sparse_reward_index", 3f);

        ball_balance_dense_reward_index = m_ResetParams.GetWithDefault("ball_balance_dense_reward_index", 0f);
        ball_balance_sparse_reward_index = m_ResetParams.GetWithDefault("ball_balance_sparse_reward_index", 2f);

        trimatch_dense_reward_index = m_ResetParams.GetWithDefault("trimatch_dense_reward_index", 14f);
        trimatch_sparse_reward_index = m_ResetParams.GetWithDefault("trimatch_sparse_reward_index", 0f);

        print_pythonparams = m_ResetParams.GetWithDefault("print_pythonparams", 0f);
        print_rewardlog = m_ResetParams.GetWithDefault("print_rewardlog", 1f);
        
    }

    void InitRewardCalculator()
    {
        pre_balldist = Vector3.Distance(target.position, ball.position);
        pre_boarddist = Vector3.Distance(board.position, board_target);
        pre_boardang = Vector3.Angle(board.up, Vector3.up);
        // previous distance
        pre_l1 = Vector3.Distance(l1.position, al1.position);
        pre_l2 = Vector3.Distance(l2.position, al2.position);
        pre_l3 = Vector3.Distance(l3.position, al3.position);
        pre_r1 = Vector3.Distance(r1.position, ar1.position);
        pre_r2 = Vector3.Distance(r2.position, ar2.position);
        pre_r3 = Vector3.Distance(r3.position, ar3.position);

        pre_hand_dist=Vector3.Distance(l1.position, al1.position) + Vector3.Distance(r1.position, ar1.position);
        pre_hand_ang= 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f) + 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);
        
    }

    void PrintPythonParams()
    {
        // general parameters
        Debug.Log("force_magnitude: " + force_magnitude);
        Debug.Log("terminate_steps: " + terminate_steps);

        Debug.Log("target_distance: " + target_distance);
        Debug.Log("target_angle: " + target_angle);
        Debug.Log("target_maintain_time: " + target_maintain_time);

        // Debug.Log("use_comfort_pose: " + use_comfort_pose);
        // Debug.Log("use_ball_balance: " + use_ball_balance);
        // Debug.Log("use_board: " + use_board);
        // Debug.Log("use_trimatch: " + use_trimatch);

        Debug.Log("ball_balance_ratio: " + ball_balance_ratio);
        Debug.Log("board_ratio: " + board_ratio);
        Debug.Log("comfort_pose_ratio: " + comfort_pose_ratio);
        Debug.Log("trimatch_ratio: " + trimatch_ratio);

        Debug.Log("cam_rot_speed: " + cam_rot_speed);
        Debug.Log("cam_look_distance: " + cam_look_distance);

        Debug.Log("observe_index: " + observe_index);
        Debug.Log("action_index: " + action_index);
        Debug.Log("use_support: " + use_support);

        Debug.Log("comfort_dense_reward_index: " + comfort_dense_reward_index);
        Debug.Log("comfort_sparse_reward_index: " + comfort_sparse_reward_index);

        Debug.Log("board_dense_reward_index: " + board_dense_reward_index);
        Debug.Log("board_sparse_reward_index: " + board_sparse_reward_index);

        Debug.Log("ball_balance_dense_reward_index: " + ball_balance_dense_reward_index);
        Debug.Log("ball_balance_sparse_reward_index: " + ball_balance_sparse_reward_index);

        Debug.Log("trimatch_dense_reward_index: " + trimatch_dense_reward_index);
        Debug.Log("trimatch_sparse_reward_index: " + trimatch_sparse_reward_index);

        Debug.Log("time_penalty_reward: " + time_penalty_reward);
    }

    
    void Observation1(VectorSensor sensor) //139-16 = 123
    {
        //3
        sensor.AddObservation(target.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(ballrigid.velocity);
        sensor.AddObservation(ballrigid.angularVelocity);

        sensor.AddObservation(ball.position - target.position);
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(boardrigid.velocity);
        sensor.AddObservation(boardrigid.angularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.position);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity);
        sensor.AddObservation(upper_arm_left.velocity);
        
        sensor.AddObservation(lower_arm_left.transform.position);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity); 
        sensor.AddObservation(lower_arm_left.velocity);
        
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity);
        sensor.AddObservation(hand_left.velocity);
        
        sensor.AddObservation(upper_arm_right.transform.position);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity); 
        sensor.AddObservation(upper_arm_right.velocity); 
        
        sensor.AddObservation(lower_arm_right.transform.position); 
        sensor.AddObservation(lower_arm_right.transform.rotation); 
        sensor.AddObservation(lower_arm_right.angularVelocity); 
        sensor.AddObservation(lower_arm_right.velocity); 
        
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity);
        sensor.AddObservation(hand_right.velocity);
    }
    
    void Observation2(VectorSensor sensor) //139 - 16 = 123
    {
        //3
        sensor.AddObservation(target.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(ballrigid.velocity);
        sensor.AddObservation(ballrigid.angularVelocity);

        sensor.AddObservation(ball.position - target.position);
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(boardrigid.velocity);
        sensor.AddObservation(boardrigid.angularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.localPosition);
        sensor.AddObservation(upper_arm_left.transform.localRotation);
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.angularVelocity));
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.velocity));
        
        sensor.AddObservation(lower_arm_left.transform.localPosition);
        sensor.AddObservation(lower_arm_left.transform.localRotation);
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.angularVelocity)); 
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.velocity));
        
        sensor.AddObservation(hand_left.transform.localPosition);
        sensor.AddObservation(hand_left.transform.localRotation);
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.angularVelocity));
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.velocity));
        
        sensor.AddObservation(upper_arm_right.transform.localPosition);
        sensor.AddObservation(upper_arm_right.transform.localRotation);
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.angularVelocity)); 
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.velocity)); 
        
        sensor.AddObservation(lower_arm_right.transform.localPosition); 
        sensor.AddObservation(lower_arm_right.transform.localRotation); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.angularVelocity)); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.velocity)); 
        
        sensor.AddObservation(hand_right.transform.localPosition);
        sensor.AddObservation(hand_right.transform.localRotation);
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.angularVelocity));
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.velocity));

        // Debug.Log(this.name + " hand position " + hand_right.transform.position.ToString("F3") + " local position " + hand_right.transform.localPosition.ToString("F3") + " inversedirection ");
        // Debug.Log(this.name + " hand local "+ hand_right.transform.InverseTransformPoint(hand_right.transform.position).ToString("F3"));
        // Debug.Log(this.name + " lower arm local" + lower_arm_right_joint.transform.InverseTransformPoint(hand_right.transform.position).ToString("F3"));
        // Debug.Log(this.name + " angularv " + hand_right.angularVelocity + " local av " + hand_right.transform.InverseTransformPoint(hand_right.angularVelocity));
        // Debug.Log(this.name + "hand-joint " + (hand_right.transform.position - hand_right_joint.transform.position).ToString("F3"));
        // Debug.Log(this.name + "hand-lowerarm " + (hand_right.transform.position - lower_arm_right.transform.position).ToString("F3"));


    }

    void Observation3(VectorSensor sensor) //139 - 16 = 123 + 3 = 126
    {
        //3
        sensor.AddObservation(target.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(ballrigid.velocity);
        sensor.AddObservation(ballrigid.angularVelocity);

        sensor.AddObservation(ball.position - target.position);
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(boardrigid.velocity);
        sensor.AddObservation(boardrigid.angularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.position);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity);
        sensor.AddObservation(upper_arm_left.velocity);
        
        sensor.AddObservation(lower_arm_left.transform.position);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity); 
        sensor.AddObservation(lower_arm_left.velocity);
        
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity);
        sensor.AddObservation(hand_left.velocity);
        
        sensor.AddObservation(upper_arm_right.transform.position);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity); 
        sensor.AddObservation(upper_arm_right.velocity); 
        
        sensor.AddObservation(lower_arm_right.transform.position); 
        sensor.AddObservation(lower_arm_right.transform.rotation); 
        sensor.AddObservation(lower_arm_right.angularVelocity); 
        sensor.AddObservation(lower_arm_right.velocity); 
        
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity);
        sensor.AddObservation(hand_right.velocity);

        sensor.AddObservation(handreach_reward);
        sensor.AddObservation(boardreach_reward);
        sensor.AddObservation(ballreach_reward);

    }

    void Observation4(VectorSensor sensor) //139 - 16 = 123 + 3 = 126
    {
        //3
        sensor.AddObservation(target.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(ballrigid.velocity);
        sensor.AddObservation(ballrigid.angularVelocity);

        sensor.AddObservation(ball.position - target.position);
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(boardrigid.velocity);
        sensor.AddObservation(boardrigid.angularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.localPosition);
        sensor.AddObservation(upper_arm_left.transform.localRotation);
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.angularVelocity));
        sensor.AddObservation(upper_arm_left.transform.InverseTransformDirection(upper_arm_left.velocity));
        
        sensor.AddObservation(lower_arm_left.transform.localPosition);
        sensor.AddObservation(lower_arm_left.transform.localRotation);
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.angularVelocity)); 
        sensor.AddObservation(lower_arm_left.transform.InverseTransformDirection(lower_arm_left.velocity));
        
        sensor.AddObservation(hand_left.transform.localPosition);
        sensor.AddObservation(hand_left.transform.localRotation);
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.angularVelocity));
        sensor.AddObservation(hand_left.transform.InverseTransformDirection(hand_left.velocity));
        
        sensor.AddObservation(upper_arm_right.transform.localPosition);
        sensor.AddObservation(upper_arm_right.transform.localRotation);
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.angularVelocity)); 
        sensor.AddObservation(upper_arm_right.transform.InverseTransformDirection(upper_arm_right.velocity)); 
        
        sensor.AddObservation(lower_arm_right.transform.localPosition); 
        sensor.AddObservation(lower_arm_right.transform.localRotation); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.angularVelocity)); 
        sensor.AddObservation(lower_arm_right.transform.InverseTransformDirection(lower_arm_right.velocity)); 
        
        sensor.AddObservation(hand_right.transform.localPosition);
        sensor.AddObservation(hand_right.transform.localRotation);
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.angularVelocity));
        sensor.AddObservation(hand_right.transform.InverseTransformDirection(hand_right.velocity));

        sensor.AddObservation(handreach_reward);
        sensor.AddObservation(boardreach_reward);
        sensor.AddObservation(ballreach_reward);

    }

    void Observation7(VectorSensor sensor) //139-16 = 123
    {
        //3
        sensor.AddObservation(target.position);
        //17
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(ballrigid.velocity);
        sensor.AddObservation(ballrigid.angularVelocity/ballrigid.maxAngularVelocity);

        sensor.AddObservation(ball.position - target.position);
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        //17
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(boardrigid.velocity);
        sensor.AddObservation(boardrigid.angularVelocity/boardrigid.maxAngularVelocity);

        sensor.AddObservation(board.position - board_target);
        sensor.AddObservation(Vector3.Distance(board.position, board_target));
        // 24
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        // sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        // sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        // sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        // sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));

        sensor.AddObservation(l1.position - al1.position);
        // sensor.AddObservation(l2.position - al2.position);
        // sensor.AddObservation(l3.position - al3.position);
        sensor.AddObservation(r1.position - ar1.position);
        // sensor.AddObservation(r2.position - ar2.position);
        // sensor.AddObservation(r3.position - ar3.position);
        // 13*6=78
        sensor.AddObservation(upper_arm_left.transform.position);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity/upper_arm_left.maxAngularVelocity);
        sensor.AddObservation(upper_arm_left.velocity);
        
        sensor.AddObservation(lower_arm_left.transform.position);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity/lower_arm_left.maxAngularVelocity); 
        sensor.AddObservation(lower_arm_left.velocity);
        
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity/hand_left.maxAngularVelocity);
        sensor.AddObservation(hand_left.velocity);
        
        sensor.AddObservation(upper_arm_right.transform.position);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity/upper_arm_right.maxAngularVelocity); 
        sensor.AddObservation(upper_arm_right.velocity); 
        
        sensor.AddObservation(lower_arm_right.transform.position); 
        sensor.AddObservation(lower_arm_right.transform.rotation); 
        sensor.AddObservation(lower_arm_right.angularVelocity/lower_arm_right.maxAngularVelocity); 
        sensor.AddObservation(lower_arm_right.velocity); 
        
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity/hand_right.maxAngularVelocity);
        sensor.AddObservation(hand_right.velocity);

        sensor.AddObservation(handreach_reward);
        sensor.AddObservation(boardreach_reward);
        sensor.AddObservation(ballreach_reward);
    }
    void SparseRewardTriMatch1()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        var rew1 = 0f;

        if (cur_l1 + cur_l2 + cur_l3 <= target_distance)
        {
            rew1 += r_reach * trimatch_ratio;
            AddReward(r_reach * trimatch_ratio);
        }

        if (cur_r1 + cur_r2 + cur_r3 <= target_distance)
        {
            rew1 += r_reach * trimatch_ratio;
            AddReward(r_reach * trimatch_ratio);
        }

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (rew1).ToString();
    }

    void SparseRewardTriMatch2()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);

        // Debug.Log("left rot: " + cur_l1rot + ", right rot: " + cur_r1rot + ", left dist: " + cur_l1 + ", right dist: " + cur_r1);
        // left rot: 0.004042327, right rot: 0.001753271, left dist: 0.01271493, right dist: 0.008624015
        var reward = 0f;
        if ((cur_l1 + cur_r1) <= target_distance && (cur_l1rot + cur_r1rot <= target_angle))
        {
            reward = 1f * trimatch_ratio;
            AddReward(reward);

            this.SetReward(-1f);
            otheragent.SetReward(-1f);

            this.EndEpisode();
            otheragent.EndEpisode();
        }

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward).ToString();

        handreach_reward = reward;
    }

    void DenseRewardTriMatch1()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        // float sdist = cur_l1 + cur_l2 + cur_l3 + cur_r1 + cur_r2 + cur_r3;
        float dist = Mathf.Pow(cur_l1, 2f)+Mathf.Pow(cur_l2, 2f)+Mathf.Pow(cur_l3, 2f)+Mathf.Pow(cur_r1, 2f)+Mathf.Pow(cur_r2, 2f)+Mathf.Pow(cur_r3, 2f);
        float reward = Mathf.Exp(-dist)*trimatch_ratio;
        // Debug.Log(this.name + " trimatch distance: " + dist);
        
        AddReward(reward);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

    }

    void DenseRewardTriMatch2()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        // cur_l2 = Vector3.Distance(l2.position, al2.position);
        // cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        // cur_r2 = Vector3.Distance(r2.position, ar2.position);
        // cur_r3 = Vector3.Distance(r3.position, ar3.position);

        // float sdist = cur_l1 + cur_l2 + cur_l3 + cur_r1 + cur_r2 + cur_r3;
        float dist = Mathf.Pow(cur_l1, 2f)+Mathf.Pow(cur_r1, 2f);

        float reward = Mathf.Exp(-dist*10f)*trimatch_ratio;
        // Debug.Log(this.name + " trimatch distance: " + dist+", " + sdist);
        
        AddReward(reward);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString();
    }
    void DenseRewardTriMatch3()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        var rew1 = 0f;
        if (cur_l1 + cur_l2 + cur_l3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * trimatch_ratio;
            AddReward(r_reach * trimatch_ratio);
        }

        if (cur_r1 + cur_r2 + cur_r3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * trimatch_ratio;
            AddReward(r_reach * trimatch_ratio);
        }

        AddReward((0.01f - cur_l1));
        AddReward((0.01f - cur_l2));
        AddReward((0.01f - cur_l3));

        AddReward((0.01f - cur_r1));
        AddReward((0.01f - cur_r2));
        AddReward((0.01f - cur_r3));

        var rew2 = 0f;
        rew2 = 0.06f - cur_l1 - cur_l2 - cur_l3 - cur_r1 - cur_r2 - cur_r3;

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (rew1 + rew2).ToString();
    }

    void DenseRewardTriMatch4()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        var rew1 = 0f;
        if (cur_l1 + cur_l2 + cur_l3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * trimatch_ratio;
            AddReward(r_reach * trimatch_ratio);
        }

        if (cur_r1 + cur_r2 + cur_r3 <= target_distance / 2.0f)
        {
            rew1 += r_reach * trimatch_ratio;
            AddReward(r_reach * trimatch_ratio);
        }

        var limit = 0.01f;
        if (cur_l1 < limit) cur_l1 = limit;
        if (cur_l2 < limit) cur_l2 = limit;
        if (cur_l3 < limit) cur_l3 = limit;
        if (cur_r1 < limit) cur_r1 = limit;
        if (cur_r2 < limit) cur_r2 = limit;
        if (cur_r3 < limit) cur_r3 = limit;

        var rew2 = 0f;
        var upper_limit = 0.005f;
        AddReward(upper_limit / cur_l1);
        AddReward(upper_limit / cur_l2);
        AddReward(upper_limit / cur_l3);

        AddReward(upper_limit / cur_r1);
        AddReward(upper_limit / cur_r2);
        AddReward(upper_limit / cur_r3);

        rew2 += upper_limit / cur_l1 + upper_limit / cur_l2 + upper_limit / cur_l3 + upper_limit / cur_r1 + upper_limit / cur_r2 + upper_limit / cur_r3;

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (rew1 + rew2).ToString();
    }

    void DenseRewardTriMatch5()
    {
        // to gurantee the reward value to be in the range of (-infinity, 1]
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        var reward = 0f ;
        reward += Mathf.Exp(-cur_l1-cur_l2-cur_l3);
        reward += Mathf.Exp(-cur_r1-cur_r2-cur_r3);

        AddReward( reward * trimatch_ratio);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward * trimatch_ratio).ToString();
    }

    void DenseRewardTriMatch6()
    {
        // to gurantee the reward value to be in the range of (-infinity, 1]
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        var reward = 0f ;
        reward += Mathf.Exp(-cur_l1*cur_l1-cur_l2*cur_l2-cur_l3*cur_l3);
        reward += Mathf.Exp(-cur_r1*cur_l1-cur_r2*cur_l2-cur_r3*cur_l3);

        AddReward( reward * trimatch_ratio);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward * trimatch_ratio).ToString();
    }

    void DenseRewardTriMatch7()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_l2 = Vector3.Distance(l2.position, al2.position);
        cur_l3 = Vector3.Distance(l3.position, al3.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        cur_r2 = Vector3.Distance(r2.position, ar2.position);
        cur_r3 = Vector3.Distance(r3.position, ar3.position);

        // float sdist = cur_l1 + cur_l2 + cur_l3 + cur_r1 + cur_r2 + cur_r3;
        float distleft = cur_l1 + cur_l2 + cur_l3;//Mathf.Pow(cur_l1, 2f)+Mathf.Pow(cur_l2, 2f)+Mathf.Pow(cur_l3, 2f);
        float distright = cur_r1 + cur_r2 + cur_r3; //Mathf.Pow(cur_r1, 2f)+Mathf.Pow(cur_r2, 2f)+Mathf.Pow(cur_r3, 2f);
        float reward = (0.5f*Mathf.Exp(-distleft) + 0.5f*Mathf.Exp(-distright))*trimatch_ratio;
        // Debug.Log(this.name + " left trimatch distance: " + distleft + ", right trimatch distance: " + distright);
        
        AddReward(reward);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

    }

    void DenseRewardTriMatch8()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);


        var r_left = Mathf.Exp(- Mathf.Pow(cur_l1, 2f) * 5f) * 0.8f + Mathf.Exp(- cur_l1rot * 20f) * 0.2f;
        var r_right = Mathf.Exp(- Mathf.Pow(cur_r1, 2f) * 5f) * 0.8f + Mathf.Exp(- cur_r1rot * 20f) * 0.2f;
        var reward = (0.5f * r_left + 0.5f * r_right) * trimatch_ratio;
        // Debug.Log(this.name + " left dist: " + cur_l1 + ", left ang: " + cur_l1rot + "left reward: " + r_left);
        
        AddReward(reward);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

        handreach_reward = reward;
    }

    void DenseRewardTriMatch9()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);


        var r_left = Mathf.Exp(- Mathf.Pow(cur_l1, 2f) * 50f) * 0.8f + Mathf.Exp(- cur_l1rot * 20f) * 0.2f;
        var r_right = Mathf.Exp(- Mathf.Pow(cur_r1, 2f) * 50f) * 0.8f + Mathf.Exp(- cur_r1rot * 20f) * 0.2f;
        var reward = (0.5f * r_left + 0.5f * r_right) * trimatch_ratio;
        // Debug.Log(this.name + " left dist reward: " + Mathf.Exp(- Mathf.Pow(cur_l1, 2f) * 50f) + ", left ang reward: " + Mathf.Exp(- cur_l1rot * 20f) + "left reward: " + r_left);
        
        AddReward(reward);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

        handreach_reward = reward;
    }

    void DenseRewardTriMatch10()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);

        var r_left = Mathf.Exp(- cur_l1*cur_l1 * 50f); //  * 0.8f + Mathf.Exp(- cur_l1rot * 20f) * 0.2f;
        var r_right = Mathf.Exp(- cur_r1*cur_r1 * 50f); // * 0.8f + Mathf.Exp(- cur_r1rot * 20f) * 0.2f;
        var reward = (0.5f * r_left + 0.5f * r_right) * trimatch_ratio;
        
        AddReward(reward);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

        handreach_reward = reward;
    }

    void DenseRewardTriMatch11()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);

        var r_dist =  Mathf.Exp(- (cur_l1*cur_l1 + cur_r1*cur_r1) * 50f);
        var r_ang = Mathf.Exp(- (cur_l1rot + cur_r1rot) * 20f);

        var reward = (r_dist * 0.8f + r_ang * 0.2f) * trimatch_ratio;

        // Debug.Log(this.name + " dist reward: " + r_dist + ", ang reward: " + r_ang + "reward: " + reward);
        
        AddReward(reward);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

        handreach_reward = reward;
    }

    void DenseRewardTriMatch12()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);

        var r_dist =  Mathf.Exp(- (cur_l1*cur_l1 + cur_r1*cur_r1) * 10f);

        var reward = r_dist* trimatch_ratio;

        if ((cur_l1 + cur_r1) < target_distance)
        {
            reward = 1f;
        }
        AddReward(reward);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

        handreach_reward = reward;

        // Debug.DrawLine(l1.position, al1.position, Color.red);
        // Debug.DrawLine(r1.position, ar1.position, Color.green);
    }

    void DenseRewardTriMatch13()
    {
        var cur_dist = Vector3.Distance(l1.position, al1.position) + Vector3.Distance(r1.position, ar1.position);
        var cur_ang = 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f) + 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);
        float reward = 0f;
        if (cur_dist<target_distance) reward += 0.5f;
        else 
        {
            if (cur_dist < pre_hand_dist) reward += 0.2f;
        }

        if (cur_ang < target_angle) reward += 0.5f;
        else
        {
            if(cur_ang < pre_hand_ang) reward += 0.2f;
        }
        pre_hand_dist = cur_dist;
        pre_hand_ang = cur_ang;

        reward *= trimatch_ratio;
        AddReward(reward);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3");

        handreach_reward = reward;
    }

    void DenseRewardTriMatch14()
    {
        cur_l1 = Vector3.Distance(l1.position, al1.position);
        var cur_l1rot = 1f - Mathf.Pow(Quaternion.Dot(l1.rotation,al1.rotation),2f);
        cur_r1 = Vector3.Distance(r1.position, ar1.position);
        var cur_r1rot = 1f - Mathf.Pow(Quaternion.Dot(r1.rotation,ar1.rotation),2f);

        var r_dist =  Mathf.Exp(- (cur_l1*cur_l1 + cur_r1*cur_r1) * 10f);
        var r_ang = Mathf.Exp(- (cur_l1rot + cur_r1rot));

        var reward = (r_dist * 0.8f + r_ang * 0.2f) * trimatch_ratio;

        // Debug.Log(this.name + " dist reward: " + r_dist.ToString("F3") + ", ang reward: " + r_ang.ToString("F3"));
        
        AddReward(reward);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward).ToString("F3") + " dist reward: " + r_dist.ToString("F3") + ", ang reward: " + r_ang.ToString("F3");

        handreach_reward = reward;
    }
    void SparseRewardBoard1()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        var reward = 0.0f;
        if (cur_boarddist < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);
        }
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward).ToString();
    }

    void SparseRewardBoard2()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = Vector3.Angle(board.up, Vector3.up);

        var reward = 0.0f;

        if (cur_boardang < target_angle && cur_boarddist < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);
        }

        if (cur_boarddist < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);
        }
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward).ToString();
    }

    void SparseRewardBoard3()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = 1f - Mathf.Pow(Quaternion.Dot(board.rotation,Quaternion.identity),2f);

        var reward = 0f;
        if (cur_boardang <= target_angle*0.5f && cur_boarddist <= target_distance*0.5f)
        {
            reward = 1f * board_ratio;
            AddReward(reward);

            this.SetReward(-1f);
            otheragent.SetReward(-1f);

            this.EndEpisode();
            otheragent.EndEpisode();
        }
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward).ToString();

        boardreach_reward = reward;
    }
    void DenseRewardBoard1()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        var reward = 0.0f;

        if (cur_boarddist < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);

            //log += "boarddist reached, ";
        }
        else
        {
            if (cur_boarddist >= pre_boarddist)
            {
                reward += -r_approch * board_ratio;
                AddReward(-r_approch * board_ratio);
            }
            else
            {
                reward += r_approch * board_ratio;
                AddReward(r_approch * board_ratio);
            }

            //log += "boarddist= " + cur_boarddist_y.ToString("F3") + ", ";
        }

        pre_boarddist = cur_boarddist;

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void DenseRewardBoard2()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        var reward = 0.0f;
        if (cur_boarddist < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);

            //log += "boarddist reached, ";
        }

        if (cur_boarddist < target_distance) cur_boarddist = target_distance;

        reward += 0.01f / cur_boarddist;
        AddReward(0.01f / cur_boarddist);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void DenseRewardBoard3()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = Vector3.Angle(board.up, Vector3.up);

        var reward = 0.0f;

        //for angle reward constraint looks like not necessary
        if (cur_boardang < target_angle && cur_boarddist < target_distance)
            {
                reward += r_reach * board_ratio;
                AddReward(r_reach * board_ratio);
            }

        if (cur_boardang < target_angle)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);

            //log += "boardangle reached, ";
        }
        else
        {
            if (cur_boardang > pre_boardang)
            {
                reward += -r_approch * board_ratio;
                AddReward(-r_approch * board_ratio);
            }
            else
            {
                reward += r_approch * board_ratio;
                AddReward(r_approch * board_ratio);
            }

            //log += "boardangle= " + cur_boardang.ToString("F1") + ", ";
        }
        pre_boardang = cur_boardang;

        if (cur_boarddist < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);

            //log += "boarddist reached, ";
        }
        else
        {
            if (cur_boarddist > pre_boarddist)
            {
                reward += -r_approch * board_ratio;
                AddReward(-r_approch * board_ratio);
            }
            else
            {
                reward += r_approch * board_ratio;
                AddReward(r_approch * board_ratio);
            }

            //log += "boarddist= " + cur_boarddist_y.ToString("F3") + ", ";
        }

        pre_boarddist = cur_boarddist;

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void DenseRewardBoard4()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        var reward = 0.0f;
        reward += Mathf.Exp(-cur_boarddist * 25f);
        AddReward(reward * board_ratio);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward*board_ratio).ToString();
    }

    void DenseRewardBoard5()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        var reward = 0.0f;
        reward += Mathf.Exp(-cur_boarddist*cur_boarddist*625f);
        AddReward(reward * board_ratio);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward*board_ratio).ToString();
    }

    void DenseRewardBoard6()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = Vector3.Angle(board.up, Vector3.up);
        var reward = 0.0f;
        // reward += Mathf.Exp(-Mathf.Pow(cur_balldist,2f) * 1000f - Mathf.Pow(cur_boardang, 0.01f));
        reward += Mathf.Exp(-Mathf.Pow(cur_boarddist, 2f) * 1000f - Mathf.Pow(cur_boardang, 2f)*0.01f);
        AddReward(reward*board_ratio);
        // Debug.Log("dist: " + Mathf.Pow(cur_boarddist, 2f) * 1000f + ", angle: " + Mathf.Pow(cur_boardang, 2f)*0.01f);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + Mathf.Pow(cur_boarddist, 2f) * 1000f + ", angleval " + Mathf.Pow(cur_boardang, 2f)*0.01f;
    }

    void DenseRewardBoard7()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = Vector3.Angle(board.up, Vector3.up);

        var reward = 0.0f;
        var r_dist = Mathf.Pow(cur_boarddist, 2f) * 10f;
        var r_ang = Mathf.Pow(cur_boardang, 2f)*0.01f;

        reward += Mathf.Exp(- r_dist - r_ang); // the ratio change here is to make the reward more sensitive to distance.
        AddReward(reward*board_ratio);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + Mathf.Pow(cur_boarddist, 2f) * 10f + ", angleval " + Mathf.Pow(cur_boardang, 2f)*0.01f;
    }

    void DenseRewardBoard8()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = Vector3.Angle(board.up, Vector3.up);

        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 10f);
        var r_ang = Mathf.Exp( - Mathf.Pow(cur_boardang, 2f) * 0.01f);

        reward += r_dist * 0.8f + r_ang * 0.2f; // the ratio change here is to make the reward more sensitive to distance.
        AddReward(reward * board_ratio);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + r_dist * 0.8f + ", angleval " + r_ang * 0.2f;
    }

    void DenseRewardBoard9() // only use position to constraint the board
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);

        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 10f);

        reward += r_dist;
        AddReward(reward * board_ratio);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + r_dist;

    }

    void DenseRewardBoard10()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);

        float x, y, z;
        x = Vector3.Angle(board.right, Vector3.right);
        y = Vector3.Angle(board.up, Vector3.up);
        z = Vector3.Angle(board.forward, Vector3.forward);
        cur_boardang = Mathf.Pow(x, 2f) + Mathf.Pow(y, 2f) + Mathf.Pow(z, 2f);

        // Debug.Log("from vector, angle difference= " + cur_boardang.ToString("F3") + ", "+ x +", "+ y +", "+ z);
        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 10f);
        var r_ang = Mathf.Exp( - cur_boardang * 0.01f);

        reward += r_dist * 0.8f + r_ang * 0.2f; // the ratio change here is to make the reward more sensitive to distance.
        AddReward(reward * board_ratio);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + r_dist * 0.8f + ", angleval " + r_ang * 0.2f;
    }

    void DenseRewardBoard11()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        
        float x,y,z;
        var v = board.eulerAngles;
        x = Mathf.Min(v.x, 360f - v.x)/180f;
        y = Mathf.Min(v.y, 360f - v.y)/180f;
        z = Mathf.Min(v.z, 360f - v.z)/180f;
        cur_boardang = Mathf.Pow(x, 2f) + Mathf.Pow(y, 2f) + Mathf.Pow(z, 2f);

        // Debug.Log("from angle, angle difference= " + cur_boardang.ToString("F3")+ ", "+ x +", "+ y +", "+ z + ", ang= " + cur_boardang);
        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 10f);
        var r_ang = Mathf.Exp( - cur_boardang* 100f);

        reward += r_dist * 0.8f + r_ang * 0.2f; // the ratio change here is to make the reward more sensitive to distance.
        AddReward(reward * board_ratio);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + r_dist * 0.8f + ", angleval " + r_ang * 0.2f;
    }

    void DenseRewardBoard12()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = 1f - Mathf.Pow(Quaternion.Dot(board.rotation,Quaternion.identity),2f);
        
        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 10f);
        var r_ang = Mathf.Exp( - cur_boardang* 100f);

        reward += r_dist * 0.8f + r_ang * 0.2f; // the ratio change here is to make the reward more sensitive to distance.
        AddReward(reward * board_ratio);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + r_dist * 0.8f + ", angleval " + r_ang * 0.2f;
    }

    void DenseRewardBoard13()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = 1f - Mathf.Pow(Quaternion.Dot(board.rotation,Quaternion.identity),2f);
        
        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 5f);
        var r_ang = Mathf.Exp( - cur_boardang* 20f);

        reward += r_dist * 0.8f + r_ang * 0.2f; // the ratio change here is to make the reward more sensitive to distance.
        AddReward(reward * board_ratio);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString();// + ", distval " + r_dist * 0.8f + ", angleval " + r_ang * 0.2f;
        // log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + cur_boarddist + ", angleval " + cur_boardang;
        boardreach_reward = reward * board_ratio;
    }

    void DenseRewardBoard14()
    {
        cur_boarddist = Vector3.Distance(board.position, board_target);
        cur_boardang = 1f - Mathf.Pow(Quaternion.Dot(board.rotation,Quaternion.identity),2f);
        
        var reward = 0.0f;
        var r_dist = Mathf.Exp(- Mathf.Pow(cur_boarddist, 2f) * 100f);
        var r_ang = Mathf.Exp( - cur_boardang* 20f);

        reward += r_dist * 0.8f + r_ang * 0.2f; // the ratio change here is to make the reward more sensitive to distance.
        AddReward(reward * board_ratio);
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + " " + (reward*board_ratio).ToString();// + ", distval " + r_dist * 0.8f + ", angleval " + r_ang * 0.2f;
        // log += ", " + mb.Name + " " + (reward*board_ratio).ToString() + ", distval " + cur_boarddist + ", angleval " + cur_boardang;
        boardreach_reward = reward * board_ratio;
    }

    void SparseRewardBallBalance1()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position);

        var reward = 0.0f;
        if (cur_balldist <= target_distance * 2.5f)
        {
            reward += r_reach * ball_balance_ratio;
            AddReward(r_reach * ball_balance_ratio); 
        }

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void SparseRewardBallBalance2()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position) ;
        var reward = 0.0f;

        if(cur_balldist <= target_distance * 0.4f)
        {
            reward = 1f * ball_balance_ratio;
            AddReward(reward);

            this.SetReward(-1f);
            otheragent.SetReward(-1f);

            this.EndEpisode();
            otheragent.EndEpisode();
        }
        
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + ", " + reward.ToString() ;//+ ", " + cur_balldist.ToString();

        ballreach_reward = reward;
    }

    void DenseRewardBallBalance1()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position);

        float reward = 0.0f;
        if (cur_balldist <= target_distance * 2.5f)
        {
            reward += r_reach * ball_balance_ratio;
            AddReward(r_reach * ball_balance_ratio);

            //log += "ball reached, ";
        }
        else
        {
            if (cur_balldist >= pre_balldist)
            {
                reward += -r_approch * ball_balance_ratio;
                AddReward(-r_approch * ball_balance_ratio);
            }
            else
            {
                reward += r_approch * ball_balance_ratio;
                AddReward(r_approch * ball_balance_ratio);
            }

            //log += "balldist= " + cur_balldist.ToString("F3") + ", ";
        }
        pre_balldist = cur_balldist;

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void DenseRewardBallBalance2()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position);

        float reward = 0.0f;
        if (cur_balldist <= target_distance * 2.5f)
        {
            reward += r_reach * ball_balance_ratio;
            AddReward(r_reach * ball_balance_ratio);

            //log += "ball reached, ";
        }

        reward += (0.3f - cur_balldist) * 0.1f;
        AddReward((0.3f - cur_balldist) * 0.1f);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void DenseRewardBallBalance3()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position);

        float reward = 0.0f;
        if (cur_balldist <= target_distance * 2.5f)
        {
            reward += r_reach * ball_balance_ratio;
            AddReward(r_reach * ball_balance_ratio);
        }

        if (cur_balldist < 0.05f) cur_balldist = 0.05f;

        reward += 0.025f / cur_balldist;
        AddReward(0.025f / cur_balldist);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void DenseRewardBallBalance4()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position);    
        var reward = 0.0f;
        reward += Mathf.Exp(-cur_balldist*5f);
        AddReward(reward * ball_balance_ratio);
        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward * ball_balance_ratio).ToString();
    }

    void DenseRewardBallBalance5()
    {
        
        cur_balldist = Vector3.Distance(target.position, ball.position) ;
        var reward = 0.0f;
        reward += Mathf.Exp(-cur_balldist*cur_balldist*25f);
        AddReward(reward * ball_balance_ratio);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + ", " + (reward * ball_balance_ratio).ToString() ;//+ ", " + cur_balldist.ToString();
    }

    void DenseRewardBallBalance6()
    {
        
        cur_balldist = Vector3.Distance(target.position, ball.position) ;
        var reward = 0.0f;
        reward += Mathf.Exp(-cur_balldist*cur_balldist*100f);
        AddReward(reward * ball_balance_ratio);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += ", " + mb.Name + ", " + (reward * ball_balance_ratio).ToString() ;//+ ", " + cur_balldist.ToString();

        ballreach_reward = reward * ball_balance_ratio;
    }
    void SparseRewardComfortPose1()
    {

    }

    void SparseRewardComfortPose2()
    {

    }

    void DenseRewardComfortPose1()
    {
        var jointdisplacement = 0.0f;

        jointdisplacement += Vector3.Distance(upper_arm_right.transform.position, jn_upper_arm_right);
        jointdisplacement += Vector3.Distance(lower_arm_right.transform.position, jn_lower_arm_right);
        jointdisplacement += Vector3.Distance(hand_right.transform.position, jn_hand_right);
        jointdisplacement += Vector3.Distance(upper_arm_left.transform.position, jn_upper_arm_left);
        jointdisplacement += Vector3.Distance(lower_arm_left.transform.position, jn_lower_arm_left);
        jointdisplacement += Vector3.Distance(hand_left.transform.position, jn_hand_left);

        var reward = 0.0f;
        reward = jointdisplacement / 4f * 0.001f * comfort_pose_ratio;
        AddReward(reward);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + reward.ToString();
    }

    void DenseRewardComfortPose2()
    {
        var jointdisplacement = 0.0f;

        jointdisplacement += Mathf.Pow(Vector3.Distance(upper_arm_right.transform.position, jn_upper_arm_right),2f);
        jointdisplacement += Mathf.Pow(Vector3.Distance(lower_arm_right.transform.position, jn_lower_arm_right),2f);
        jointdisplacement += Mathf.Pow(Vector3.Distance(hand_right.transform.position, jn_hand_right),2f);
        jointdisplacement += Mathf.Pow(Vector3.Distance(upper_arm_left.transform.position, jn_upper_arm_left),2f);
        jointdisplacement += Mathf.Pow(Vector3.Distance(lower_arm_left.transform.position, jn_lower_arm_left),2f);
        jointdisplacement += Mathf.Pow(Vector3.Distance(hand_left.transform.position, jn_hand_left),2f);

        var reward = 0.0f;
        reward += Mathf.Exp(-jointdisplacement);
        AddReward(reward*comfort_pose_ratio);

        // Debug.Log("jointdisplacement= " + jointdisplacement + ", reward= " + reward);

        MethodBase mb = MethodBase.GetCurrentMethod();
        log += " " + mb.Name + " " + (reward*comfort_pose_ratio).ToString();
    }
    public Transform FindChildObjectwithName(Transform parent, string name)
    {
        for (var i = 0; i < parent.childCount; i++)
        {
            var child = parent.GetChild(i);
            if (child.name == name)
            {
                return child;
            }
            if (child.childCount > 0)
            {
                var found = FindChildObjectwithName(child, name);
                if (found != null)
                {
                    return found;
                }
            }
        }
        return null;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {

        float torqueforce = force_magnitude;

        ActionSegment<float> actions = actionsOut.ContinuousActions;

        actions[0] = 0f;
        actions[1] = 0f;
        actions[2] = 0f;
        actions[3] = 0f;
        actions[4] = 0f;
        actions[5] = 0f;
        actions[6] =0f;
        actions[7]=0f;
        actions[8]=0f;
        actions[9]=0f;
        actions[10]=0f;
        actions[11]=0f;
        actions[12]=0f;
        actions[13]=0f;

        if (Input.GetKey(KeyCode.Q)) actions[0] = 1;
        if (Input.GetKey(KeyCode.A)) actions[0] = -1;
        if (Input.GetKey(KeyCode.W)) actions[1] = 1;
        if (Input.GetKey(KeyCode.S)) actions[1] = -1;
        if (Input.GetKey(KeyCode.E)) actions[2] = 1;
        if (Input.GetKey(KeyCode.D)) actions[2] = -1;
        if (Input.GetKey(KeyCode.R)) actions[3] = 1;
        if (Input.GetKey(KeyCode.F)) actions[3] = -1;
        if (Input.GetKey(KeyCode.T)) actions[4] = 1;
        if (Input.GetKey(KeyCode.G)) actions[4] = -1;
        if (Input.GetKey(KeyCode.Y)) actions[5] = 1;
        if (Input.GetKey(KeyCode.H)) actions[5] = -1;
        if (Input.GetKey(KeyCode.U)) actions[6] = 1;
        if (Input.GetKey(KeyCode.J)) actions[6] = -1;
        if (Input.GetKey(KeyCode.I)) actions[7] = 1;
        if (Input.GetKey(KeyCode.K)) actions[7] = -1;
        if (Input.GetKey(KeyCode.O)) actions[8] = 1;
        if (Input.GetKey(KeyCode.L)) actions[8] = -1;
        if (Input.GetKey(KeyCode.Z)) actions[9] = 1;
        if (Input.GetKey(KeyCode.X)) actions[9] = -1;
        if (Input.GetKey(KeyCode.C)) actions[10] = 1;
        if (Input.GetKey(KeyCode.V)) actions[10] = -1;
        if (Input.GetKey(KeyCode.B)) actions[11] = 1;
        if (Input.GetKey(KeyCode.N)) actions[11] = -1;
        if (Input.GetKey(KeyCode.M)) actions[12] = 1;
        if (Input.GetKey(KeyCode.P)) actions[12] = -1;
        if (Input.GetKey(KeyCode.LeftArrow)) actions[13] = 1;
        if (Input.GetKey(KeyCode.RightArrow)) actions[13] = -1;
        
        TestActionFun(actions);
    }

    public void TestActionFun(ActionSegment<float> action)
    {
       if (action[0] > 0) Debug.Log(this.name + " Q pressed, lefthold X joint force increased" + " " + action[0] + " " + l1.position.ToString("F3"));
       if (action[0] < 0) Debug.Log(this.name + " A pressed, lefthold X joint force decreased" + " " + action[0] + " " + l1.position.ToString("F3"));
       if (action[1] > 0) Debug.Log(this.name + " W pressed, lefthold Y joint force increased" + " " + action[1] + " " + l1.position.ToString("F3"));
       if (action[1] < 0) Debug.Log(this.name + " S pressed, lefthold Y joint force decreased" + " " + action[1] + " " + l1.position.ToString("F3"));
       if (action[2] > 0) Debug.Log(this.name + " E pressed, lefthold Z joint force increased" + " " + action[2] + " " + l1.position.ToString("F3"));
       if (action[2] < 0) Debug.Log(this.name + " D pressed, lefthold Z joint force decreased" + " " + action[2] + " " + l1.position.ToString("F3"));
       if (action[3] > 0) Debug.Log(this.name + " R pressed, righthold X joint force increased" + " " + action[3] + " " + r1.position.ToString("F3"));
       if (action[3] < 0) Debug.Log(this.name + " F pressed, righthold X joint force decreased" + " " + action[3] + " " + r1.position.ToString("F3"));
       if (action[4] > 0) Debug.Log(this.name + " T pressed, righthold Y joint force increased" + " " + action[4] + " " + r1.position.ToString("F3"));
       if (action[4] < 0) Debug.Log(this.name + " G pressed, righthold Y joint force decreased" + " " + action[4] + " " + r1.position.ToString("F3"));
       if (action[5] > 0) Debug.Log(this.name + " Y pressed, righthold Z joint force increased" + " " + action[5] + " " + r1.position.ToString("F3"));
       if (action[5] < 0) Debug.Log(this.name + " H pressed, righthold Z joint force decreased" + " " + action[5] + " " + r1.position.ToString("F3"));
       
    }
}


