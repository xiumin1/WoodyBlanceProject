using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class WoodyAgent_tmhc : Agent
{
    //------------------------------------------------------------------------------------------
    // the two agents init position and the whole environment info will be initilized and reset in the woodyarea script for simplicity
    //public bool invertagent; // this bool value will help to differenciate the two agent

    //GameObject agent; // source prefab and the generated agent

    WoodyArea woodyarea;
    Transform board, target, ball;
    Rigidbody boardrigid, ballrigid;

    Rigidbody upper_arm_right;
    Rigidbody lower_arm_right;
    Rigidbody hand_right;
    Transform hand_right_end;
    Rigidbody upper_arm_left;
    Rigidbody lower_arm_left;
    Rigidbody hand_left;
    Transform hand_left_end;

    int action_count = 18;
    Material crosswin, crosslose;
    public Transform l1, l2, l3, r1, r2, r3; // the three holding points on the board
    Transform al1, al2, al3, ar1, ar2, ar3;// the three holding points on the agent hands
    // use to calculate rewards
    float cur_l1, cur_l2, cur_l3, cur_r1, cur_r2, cur_r3; // the current distance of holding point between hand and board
    float pre_l1, pre_l2, pre_l3, pre_r1, pre_r2, pre_r3;
    float pre_balldist, cur_balldist; // to award the reward when the ball arrives to the target location
    float pre_boarddist_y, cur_boarddist_y, pre_boardang, cur_boardang; // use to calculate reward for board balance in position and angle
    float board_target_y; // the target position where the board supposed to stay at balance status
    // use to access parameters from outside of C# and Unity
    EnvironmentParameters m_ResetParams;
    float force_magnitude = 0.0f;  // the force magnitude adds to each joint
    float terminate_steps = 0;     // the max steps needs for each episode, use to calculate the time penalty

    float target_distance = 0.02f; // the target distance bewtween hand to holding location, 
    float target_angle = 5f;       // the target angle distance between hand to up direction

    // 0.0 means not add this constraint, 1.0 means add it
    [HideInInspector]
    public float use_comfort_pose = 0.0f; // use to check if need to add the comfort posture constraint into reward for training.
    [HideInInspector]
    public float use_ball_balance = 0.0f; // use to check if need to add ball balance reward to the training
    [HideInInspector]
    public float use_board = 0.0f; // use to check if need to add board height reach reward to the training
    [HideInInspector]
    public float use_trimatch = 0.0f; // use to check if decide to use trimatch reward to replace: hand angle, hand distance, palm touch board reward functions.
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
    string log = ""; // use to hold the log information for check in standalone exe

    Vector3 jn_upper_arm_right, jn_lower_arm_right, jn_hand_right, jn_upper_arm_left, jn_lower_arm_left, jn_hand_left;

    int leftcount = 0;
    int rightcount = 0;

    public override void Initialize()
    {
        woodyarea = GameObject.Find("WoodyArea").GetComponent<WoodyArea>();
        woodyarea.AreaInit();

        board = woodyarea.board;
        target = woodyarea.target;
        ball = woodyarea.ball;
        boardrigid = board.gameObject.GetComponent<Rigidbody>();
        ballrigid = ball.gameObject.GetComponent<Rigidbody>();

        board_target_y = board.position.y + 0.1f;

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

        SetResetParameters();
        // initialize the pre values for distance or angle calculation for reward
        al1 = FindChildObjectwithName(this.transform, "l1");
        al2 = FindChildObjectwithName(this.transform, "l2");
        al3 = FindChildObjectwithName(this.transform, "l3");

        ar1 = FindChildObjectwithName(this.transform, "r1");
        ar2 = FindChildObjectwithName(this.transform, "r2");
        ar3 = FindChildObjectwithName(this.transform, "r3");

        InitRewardCalculator();
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
    }

    public override void CollectObservations(VectorSensor sensor) // 6+24+2+13*6+7*2=124
    {
        if (observe_index == 1)
        {
            Observation1(sensor);
        }

        if (observe_index == 2)
        {
            Observation2(sensor);
        }  
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var vectoraction = actionBuffers.ContinuousActions;
        for (var i = 0; i < action_count; i++)
        {
            vectoraction[i] = Mathf.Clamp(vectoraction[i], -1, 1f) * force_magnitude;
            //debug.log(i.tostring() + "th action= " + actions[i].tostring());
        }

        upper_arm_right_torque = new Vector3(vectoraction[0], vectoraction[1] / 100, vectoraction[2]);
        lower_arm_right_torque = new Vector3(vectoraction[3], vectoraction[4] / 100, vectoraction[5]);
        hand_right_torque = new Vector3(vectoraction[6], vectoraction[7] / 100, vectoraction[8]);

        upper_arm_left_torque = new Vector3(vectoraction[9], vectoraction[10] / 100, vectoraction[11]);
        lower_arm_left_torque = new Vector3(vectoraction[12], vectoraction[13] / 100, vectoraction[14]);
        hand_left_torque = new Vector3(vectoraction[15], vectoraction[16] / 100, vectoraction[17]);

        upper_arm_right.AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_right.AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
        hand_right.AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

        upper_arm_left.AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
        lower_arm_left.AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
        hand_left.AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

        log = "";

        ////add reward for lower energy of the arms
        if (use_comfort_pose == 1.0) GetRewardComfortPose();

        if (use_board == 1.0) GetRewardBoard();

        if (use_ball_balance == 1.0) GetRewardBallBalance();
         
        if (use_trimatch == 1.0) GetRewardTriMatch();

        AddReward(-1 / terminate_steps);

        if (print_pythonparams == 1f) PrintPythonParams();

        if (print_rewardlog == 1f) Debug.Log(this.name +"----" +  log );

        ConnectBoard();

        //Debug.DrawLine(l1.position, al1.position, Color.red);
        //Debug.DrawLine(l2.position, al2.position, Color.green);
        //Debug.DrawLine(l3.position, al3.position, Color.yellow);

        //Debug.DrawLine(r1.position, ar1.position, Color.red);
        //Debug.DrawLine(r2.position, ar2.position, Color.green);
        //Debug.DrawLine(r3.position, ar3.position, Color.yellow);
    }

    public void ConnectBoard()
    {
        if (leftcount == 1)
        {
            FixedJoint joint = board.gameObject.AddComponent<FixedJoint>();
            joint.connectedBody = hand_left.transform.GetComponent<Rigidbody>();
            joint.breakForce = Mathf.Infinity;
            leftcount += 1;
        }

        if (rightcount == 1)
        {
            FixedJoint joint = board.gameObject.AddComponent<FixedJoint>();
            joint.connectedBody = hand_right.transform.GetComponent<Rigidbody>();
            joint.breakForce = Mathf.Infinity;
            rightcount += 1;
        } 
    }

    public void GetRewardTriMatch()
    {
        // for sparse reward
        if (trimatch_sparse_reward_index == 1) SparseRewardTriMatch1();

        if (trimatch_sparse_reward_index == 2) SparseRewardTriMatch2();

        // for dense reward
        if (trimatch_dense_reward_index == 1) DenseRewardTriMatch1();

        if (trimatch_dense_reward_index == 2) DenseRewardTriMatch2();

        if (trimatch_dense_reward_index == 3) DenseRewardTriMatch3();
    }

    public void GetRewardBallBalance()
    {
        // for sparse reward
        if (ball_balance_sparse_reward_index == 1) SparseRewardBallBalance1();

        if (ball_balance_sparse_reward_index == 2) SparseRewardBallBalance2();

        // for dense reward
        if (ball_balance_dense_reward_index == 1) DenseRewardBallBalance1();

        if (ball_balance_dense_reward_index == 2) DenseRewardBallBalance2();

        if (ball_balance_dense_reward_index == 3) DenseRewardBallBalance3();
    }

    public void GetRewardBoard()
    {
        if (board_sparse_reward_index == 1) SparseRewardBoard1();

        if (board_sparse_reward_index == 2) SparseRewardBoard2();

        if (board_dense_reward_index == 1) DenseRewardBoard1();

        if (board_dense_reward_index == 2) DenseRewardBoard2();

        //if (board_dense_reward_index == 3) DenseRewardBoard3();
    }

    public void GetRewardComfortPose()
    {
        if (comfort_sparse_reward_index == 1) SparseRewardComfortPose1();

        if (comfort_sparse_reward_index == 2) SparseRewardComfortPose2();

        if (comfort_dense_reward_index == 1) DenseRewardComfortPose1();

        if (comfort_dense_reward_index == 2) DenseRewardComfortPose2();

    }

    public override void OnEpisodeBegin()
    {
        woodyarea.AreaReset();
        woodyarea.AgentInitPoseReset(this.name);
        //reset the force ratio here as a accessible paramters in python api
        SetResetParameters();
        InitRewardCalculator();

        leftcount = 0;
        rightcount = 0;
        FixedJoint[] joints = board.gameObject.GetComponents<FixedJoint>();
        foreach (FixedJoint j in joints)
        {
            Destroy(j);
        }
    }

    void SetResetParameters()
    {
        // general parameters
        force_magnitude = m_ResetParams.GetWithDefault("force_magnitude", 500.0f);
        terminate_steps = m_ResetParams.GetWithDefault("terminate_steps", 500f);

        // target distance and anlge value, could also be used for curriculum learning purpose
        target_distance = m_ResetParams.GetWithDefault("target_distance", 0.02f);
        target_angle = m_ResetParams.GetWithDefault("target_angle", 5f);

        // control parameters for which reward to use for training    
        use_comfort_pose = m_ResetParams.GetWithDefault("use_comfort_pose", 0.0f);
        use_ball_balance = m_ResetParams.GetWithDefault("use_ball_balance", 1.0f);
        use_board = m_ResetParams.GetWithDefault("use_board", 1.0f);
        use_trimatch = m_ResetParams.GetWithDefault("use_trimatch", 1.0f);

        // reward ratio parameters for controlling the weight ratio of each reward 
        ball_balance_ratio = m_ResetParams.GetWithDefault("ball_balance_ratio", 2f);
        board_ratio = m_ResetParams.GetWithDefault("board_ratio", 2f);
        comfort_pose_ratio = m_ResetParams.GetWithDefault("comfort_pose_ratio", 1f);
        trimatch_ratio = m_ResetParams.GetWithDefault("trimatch_ratio", 1f);

        // for camera
        cam_rot_speed = m_ResetParams.GetWithDefault("cam_rot_speed", 0.1f);
        cam_look_distance = m_ResetParams.GetWithDefault("cam_look_distance", 1.5f);

        // to use a flag index to decide which part of code to use, separate sparse reward and dense reward
        observe_index = m_ResetParams.GetWithDefault("observe_index", 1f);

        comfort_dense_reward_index = m_ResetParams.GetWithDefault("comfort_dense_reward_index", 0f);
        comfort_sparse_reward_index = m_ResetParams.GetWithDefault("comfort_sparse_reward_index", 0f);

        board_dense_reward_index = m_ResetParams.GetWithDefault("board_dense_reward_index", 1f);
        board_sparse_reward_index = m_ResetParams.GetWithDefault("board_sparse_reward_index", 0f);

        ball_balance_dense_reward_index = m_ResetParams.GetWithDefault("ball_balance_dense_reward_index", 1f);
        ball_balance_sparse_reward_index = m_ResetParams.GetWithDefault("ball_balance_sparse_reward_index", 0f);

        trimatch_dense_reward_index = m_ResetParams.GetWithDefault("trimatch_dense_reward_index", 1f);
        trimatch_sparse_reward_index = m_ResetParams.GetWithDefault("trimatch_sparse_reward_index", 0f);

        print_pythonparams = m_ResetParams.GetWithDefault("print_pythonparams", 0f);
        print_rewardlog = m_ResetParams.GetWithDefault("print_rewardlog", 1f);
    }

    void InitRewardCalculator()
    {
        pre_balldist = Vector3.Distance(target.position, ball.position);

        pre_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        pre_boardang = Vector3.Angle(board.up, Vector3.up);

        // previous distance
        pre_l1 = Vector3.Distance(l1.position, al1.position);
        pre_l2 = Vector3.Distance(l2.position, al2.position);
        pre_l3 = Vector3.Distance(l3.position, al3.position);
        pre_r1 = Vector3.Distance(r1.position, ar1.position);
        pre_r2 = Vector3.Distance(r2.position, ar2.position);
        pre_r3 = Vector3.Distance(r3.position, ar3.position);
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

        float torqueforce = 10f;
        var actions = actionsOut.ContinuousActions;

        if (Input.GetKey(KeyCode.Q)) actions[0] = torqueforce;
        if (Input.GetKey(KeyCode.A)) actions[0] = -torqueforce;
        if (Input.GetKey(KeyCode.W)) actions[1] = torqueforce;
        if (Input.GetKey(KeyCode.S)) actions[1] = -torqueforce;
        if (Input.GetKey(KeyCode.E)) actions[2] = torqueforce;
        if (Input.GetKey(KeyCode.D)) actions[2] = -torqueforce;
        if (Input.GetKey(KeyCode.R)) actions[3] = torqueforce;
        if (Input.GetKey(KeyCode.F)) actions[3] = -torqueforce;
        if (Input.GetKey(KeyCode.T)) actions[4] = torqueforce;
        if (Input.GetKey(KeyCode.G)) actions[4] = -torqueforce;
        if (Input.GetKey(KeyCode.Y)) actions[5] = torqueforce;
        if (Input.GetKey(KeyCode.H)) actions[5] = -torqueforce;
        if (Input.GetKey(KeyCode.U)) actions[6] = torqueforce;
        if (Input.GetKey(KeyCode.J)) actions[6] = -torqueforce;
        if (Input.GetKey(KeyCode.I)) actions[7] = torqueforce;
        if (Input.GetKey(KeyCode.K)) actions[7] = -torqueforce;
        if (Input.GetKey(KeyCode.O)) actions[8] = torqueforce;
        if (Input.GetKey(KeyCode.L)) actions[8] = -torqueforce;
        if (Input.GetKey(KeyCode.Z)) actions[9] = torqueforce;
        if (Input.GetKey(KeyCode.X)) actions[9] = -torqueforce;
        if (Input.GetKey(KeyCode.C)) actions[10] = torqueforce;
        if (Input.GetKey(KeyCode.V)) actions[10] = -torqueforce;
        if (Input.GetKey(KeyCode.B)) actions[11] = torqueforce;
        if (Input.GetKey(KeyCode.N)) actions[11] = -torqueforce;
        if (Input.GetKey(KeyCode.M)) actions[12] = torqueforce;
        if (Input.GetKey(KeyCode.P)) actions[12] = -torqueforce;
        if (Input.GetKey(KeyCode.LeftArrow)) actions[13] = torqueforce;
        if (Input.GetKey(KeyCode.RightArrow)) actions[13] = -torqueforce;

    }

    void PrintPythonParams()
    {
        // general parameters
        Debug.Log("force_magnitude: " + force_magnitude);
        Debug.Log("terminate_steps: " + terminate_steps);

        Debug.Log("target_distance: " + target_distance);
        Debug.Log("target_angle: " + target_angle);

        Debug.Log("use_comfort_pose: " + use_comfort_pose);
        Debug.Log("use_ball_balance: " + use_ball_balance);
        Debug.Log("use_board: " + use_board);
        Debug.Log("use_trimatch: " + use_trimatch);

        Debug.Log("ball_balance_ratio: " + ball_balance_ratio);
        Debug.Log("board_ratio: " + board_ratio);
        Debug.Log("comfort_pose_ratio: " + comfort_pose_ratio);
        Debug.Log("trimatch_ratio: " + trimatch_ratio);

        Debug.Log("cam_rot_speed: " + cam_rot_speed);
        Debug.Log("cam_look_distance: " + cam_look_distance);

        Debug.Log("observe_index: " + observe_index);

        Debug.Log("comfort_dense_reward_index: " + comfort_dense_reward_index);
        Debug.Log("comfort_sparse_reward_index: " + comfort_sparse_reward_index);

        Debug.Log("board_dense_reward_index: " + board_dense_reward_index);
        Debug.Log("board_sparse_reward_index: " + board_sparse_reward_index);

        Debug.Log("ball_balance_dense_reward_index: " + ball_balance_dense_reward_index);
        Debug.Log("ball_balance_sparse_reward_index: " + ball_balance_sparse_reward_index);

        Debug.Log("trimatch_dense_reward_index: " + trimatch_dense_reward_index);
        Debug.Log("trimatch_sparse_reward_index: " + trimatch_sparse_reward_index);
    }

    void Observation1(VectorSensor sensor)
    {
        //6
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));
        // 7*3+3=24 //30
        sensor.AddObservation(board.position);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(ball.position);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(target.position);
        sensor.AddObservation(target.rotation);
        sensor.AddObservation(ballrigid.velocity);
        //2
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        sensor.AddObservation(Mathf.Abs(board.position.y - board_target_y));
        //7+6=13 //45
        sensor.AddObservation(upper_arm_left.transform.position);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity);
        sensor.AddObservation(upper_arm_left.velocity);
        //13 //58
        sensor.AddObservation(lower_arm_left.transform.position);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity); //68
        sensor.AddObservation(lower_arm_left.velocity);//71
                                                       //13 //71
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity);
        sensor.AddObservation(hand_left.velocity);
        //7 //78
        sensor.AddObservation(hand_left_end.position);
        sensor.AddObservation(hand_left_end.rotation);
        //13 //91
        sensor.AddObservation(upper_arm_right.transform.position);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity); //101
        sensor.AddObservation(upper_arm_right.velocity); //104
                                                         //13 //104
        sensor.AddObservation(lower_arm_right.transform.position); //107
        sensor.AddObservation(lower_arm_right.transform.rotation); //111
        sensor.AddObservation(lower_arm_right.angularVelocity); //114
        sensor.AddObservation(lower_arm_right.velocity); //117
                                                         //13 //117
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity);
        sensor.AddObservation(hand_right.velocity);
        //7 //124
        sensor.AddObservation(hand_right_end.position);
        sensor.AddObservation(hand_right_end.rotation);
    }

    void Observation2(VectorSensor sensor)
    {
        //6
        sensor.AddObservation(Vector3.Distance(l1.position, al1.position));
        sensor.AddObservation(Vector3.Distance(l2.position, al2.position));
        sensor.AddObservation(Vector3.Distance(l3.position, al3.position));
        sensor.AddObservation(Vector3.Distance(r1.position, ar1.position));
        sensor.AddObservation(Vector3.Distance(r2.position, ar2.position));
        sensor.AddObservation(Vector3.Distance(r3.position, ar3.position));
        // 7*3+3=24 //30
        sensor.AddObservation(board.position / 2.0f);
        sensor.AddObservation(board.rotation);
        sensor.AddObservation(ball.position / 2.0f);
        sensor.AddObservation(ball.rotation);
        sensor.AddObservation(target.position / 2.0f);
        sensor.AddObservation(target.rotation);
        sensor.AddObservation(ballrigid.velocity);
        //2
        sensor.AddObservation(Vector3.Distance(ball.position, target.position));
        sensor.AddObservation(Mathf.Abs(board.position.y - board_target_y) / 2.0f);
        //7+6=13 //45
        sensor.AddObservation(upper_arm_left.transform.position / 2.0f);
        sensor.AddObservation(upper_arm_left.transform.rotation);
        sensor.AddObservation(upper_arm_left.angularVelocity / 20.0f);
        sensor.AddObservation(upper_arm_left.velocity / 2.0f);
        //13 //58
        sensor.AddObservation(lower_arm_left.transform.position / 2.0f);
        sensor.AddObservation(lower_arm_left.transform.rotation);
        sensor.AddObservation(lower_arm_left.angularVelocity / 20.0f); //68
        sensor.AddObservation(lower_arm_left.velocity / 2.0f);//71
                                                              //13 //71
        sensor.AddObservation(hand_left.transform.position);
        sensor.AddObservation(hand_left.transform.rotation);
        sensor.AddObservation(hand_left.angularVelocity / 20.0f);
        sensor.AddObservation(hand_left.velocity / 2.0f);
        //7 //78
        sensor.AddObservation(hand_left_end.position);
        sensor.AddObservation(hand_left_end.rotation);
        //13 //91
        sensor.AddObservation(upper_arm_right.transform.position / 2.0f);
        sensor.AddObservation(upper_arm_right.transform.rotation);
        sensor.AddObservation(upper_arm_right.angularVelocity / 20.0f); //101
        sensor.AddObservation(upper_arm_right.velocity / 2.0f); //104
                                                                //13 //104
        sensor.AddObservation(lower_arm_right.transform.position / 2.0f); //107
        sensor.AddObservation(lower_arm_right.transform.rotation); //111
        sensor.AddObservation(lower_arm_right.angularVelocity / 20.0f); //114
        sensor.AddObservation(lower_arm_right.velocity / 2.0f); //117
                                                                //13 //117
        sensor.AddObservation(hand_right.transform.position);
        sensor.AddObservation(hand_right.transform.rotation);
        sensor.AddObservation(hand_right.angularVelocity / 20.0f);
        sensor.AddObservation(hand_right.velocity / 2.0f);
        //7 //124
        sensor.AddObservation(hand_right_end.position);
        sensor.AddObservation(hand_right_end.rotation);
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

        log += " SparseRewardTriMatch1: " + rew1;
    }

    void SparseRewardTriMatch2()
    {

    }

    void DenseRewardTriMatch1()
    {
        var rew1 = 0f;
        var rew2 = 0f;

        if (leftcount < 1)
        {
            cur_l1 = Vector3.Distance(l1.position, al1.position);
            cur_l2 = Vector3.Distance(l2.position, al2.position);
            cur_l3 = Vector3.Distance(l3.position, al3.position);

            if (cur_l1 + cur_l2 + cur_l3 < target_distance )
            {
                rew1 += r_reach * trimatch_ratio;
                AddReward(r_reach * trimatch_ratio);

                leftcount += 1;
            }
            else
            {
                if (cur_l1 < pre_l1) { rew2 += r_approch * trimatch_ratio; AddReward(r_approch * trimatch_ratio); }
                else { rew2 += -r_approch * trimatch_ratio; AddReward(-r_approch * trimatch_ratio); }

                if (cur_l2 < pre_l2) { rew2 += r_approch * trimatch_ratio; AddReward(r_approch * trimatch_ratio); }
                else { rew2 += -r_approch * trimatch_ratio; AddReward(-r_approch * trimatch_ratio); }

                if (cur_l3 < pre_l3) { rew2 += r_approch * trimatch_ratio; AddReward(r_approch * trimatch_ratio); }
                else { rew2 += -r_approch * trimatch_ratio; AddReward(-r_approch * trimatch_ratio); }
            }

            pre_l1 = cur_l1;
            pre_l2 = cur_l2;
            pre_l3 = cur_l3;
        }
        
        if(rightcount < 1)
        {
            cur_r1 = Vector3.Distance(r1.position, ar1.position);
            cur_r2 = Vector3.Distance(r2.position, ar2.position);
            cur_r3 = Vector3.Distance(r3.position, ar3.position);

            if (cur_r1 + cur_r2 + cur_r3 < target_distance)
            {
                rew1 += r_reach * trimatch_ratio;
                AddReward(r_reach * trimatch_ratio);
                
                rightcount += 1;
            }
            else
            {
                if (cur_r1 < pre_r1) { rew2 += r_approch * trimatch_ratio; AddReward(r_approch * trimatch_ratio); }
                else { rew2 += -r_approch * trimatch_ratio; AddReward(-r_approch * trimatch_ratio); }

                if (cur_r2 < pre_r2) { rew2 += r_approch * trimatch_ratio; AddReward(r_approch * trimatch_ratio); }
                else { rew2 += -r_approch * trimatch_ratio; AddReward(-r_approch * trimatch_ratio); }

                if (cur_r3 < pre_r3) { rew2 += r_approch * trimatch_ratio; AddReward(r_approch * trimatch_ratio); }
                else { rew2 += -r_approch * trimatch_ratio; AddReward(-r_approch * trimatch_ratio); }
            }

            pre_r1 = cur_r1;
            pre_r2 = cur_r2;
            pre_r3 = cur_r3;
        }
        
        log += " DenseRewardTriMatch1: " + (rew1 + rew2).ToString();
    }

    void DenseRewardTriMatch2()
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

        log += " DenseRewardTriMatch2: " + (rew1 + rew2).ToString();
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

        log += " DenseRewardTriMatch3: " + (rew1 + rew2).ToString();
    }

    void SparseRewardBoard1()
    {
        cur_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        var reward = 0.0f;
        if (cur_boarddist_y < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);
        }
        log += " SparseRewardBoard1: " + reward;
    }

    void SparseRewardBoard2()
    {
        cur_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        cur_boardang = Vector3.Angle(board.up, Vector3.up);

        var reward = 0.0f;

        if (cur_boardang < target_angle && cur_boarddist_y < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);
        }

        if (cur_boarddist_y < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);
        }
        log += " SparseRewardBoard2: " + reward; 
    }

    void DenseRewardBoard1()
    {
        cur_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        var reward = 0.0f;

        if (cur_boarddist_y < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);

            //log += "boarddist reached, ";
        }
        else
        {
            if (cur_boarddist_y >= pre_boarddist_y)
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

        pre_boarddist_y = cur_boarddist_y;

        log += " DenseRewardBoard1: " + reward.ToString();
    }

    void DenseRewardBoard2()
    {
        cur_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        var reward = 0.0f;
        if (cur_boarddist_y < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);

            //log += "boarddist reached, ";
        }

        if (cur_boarddist_y < target_distance) cur_boarddist_y = target_distance;

        reward += 0.01f / cur_boarddist_y;
        AddReward(0.01f / cur_boarddist_y);

        log += " DenseRewardBoard2: " + reward.ToString();
    }

    void DenseRewardBoard3()
    {
        cur_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
        cur_boardang = Vector3.Angle(board.up, Vector3.up);

        var reward = 0.0f;

        //for angle reward constraint looks like not necessary
        if (cur_boardang < target_angle && cur_boarddist_y < target_distance)
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

        if (cur_boarddist_y < target_distance)
        {
            reward += r_reach * board_ratio;
            AddReward(r_reach * board_ratio);

            //log += "boarddist reached, ";
        }
        else
        {
            if (cur_boarddist_y > pre_boarddist_y)
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

        pre_boarddist_y = cur_boarddist_y;

        log += " DenseRewardBoard3: " + reward.ToString();
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
        log += " SparseRewardBallBalance1: " + reward;
    }

    void SparseRewardBallBalance2()
    {

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

        log += " DenseRewardBallBalance1: " + reward.ToString();
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

        log += " DenseRewardBallBalance2: " + reward.ToString();
    }

    void DenseRewardBallBalance3()
    {
        cur_balldist = Vector3.Distance(target.position, ball.position);

        float reward = 0.0f;
        if (cur_balldist <= target_distance * 2.5f)
        {
            reward += r_reach * ball_balance_ratio;
            AddReward(r_reach * ball_balance_ratio);

            //log += "ball reached, ";
        }

        if (cur_balldist < 0.05f) cur_balldist = 0.05f;

        reward += 0.025f / cur_balldist;
        AddReward(0.025f / cur_balldist);

        log += " DenseRewardBallBalance3: " + reward.ToString();
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
        AddReward(jointdisplacement / 4f * 0.001f * comfort_pose_ratio);

        log += " DenseRewardComfortPose1: " + reward.ToString();
    }

    void DenseRewardComfortPose2()
    {

    }
}


