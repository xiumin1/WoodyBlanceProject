//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;
//using UnityEngine.UI;
//using Unity.MLAgents;
//using Unity.MLAgents.Actuators;
//using Unity.MLAgents.Sensors;

//public class WoodyAgent1 : Agent
//{
//    //------------------------------------------------------------------------------------------
//    // the two agents init position and the whole environment info will be initilized and reset in the woodyarea script for simplicity
//    public bool invertagent; // this bool value will help to differenciate the two agent

//    GameObject woodysource, agent; // source prefab and the generated agent

//    WoodyArea woodyarea;
//    Transform board, target, ball;
//    Rigidbody boardrigid, ballrigid;

//    private Dictionary<string, Rigidbody> joints = new Dictionary<string, Rigidbody>(); // to store the 6 joints of two arms
//    private Dictionary<string, Transform> limbends = new Dictionary<string, Transform>();
//    private List<string> jointstag = new List<string>();

//    int action_count = 18;

//    Material crosswin, crosslose;

//    //private Vector3 agentfacedir; // the facing directioin of the current agent
//    public Transform lefthold, righthold; // use to set the target hold location for both hands

//    // public Transform leftcollider, rightcollider;

//    [HideInInspector]
//    public bool checkdone = false;

//    [HideInInspector]
//    public int countreach = 0; // use to check the success terminate condition

//    // use to calculate rewards
//    float pre_leftdist, pre_rightdist, cur_leftdist, cur_rightdist; // to calculate reward for hands position
//    float pre_leftang, pre_rightang, cur_leftang, cur_rightang; // to calculate reward for palm rotation
//    float pre_boarddist_y, cur_boarddist_y, pre_boardang, cur_boardang; // use to calculate reward for board balance in position and angle
//    float pre_balldist, cur_balldist; // to award the reward when the ball arrives to the target location

//    RaycastHit lefthit, righthit; // use to detect if the hand palm facing the board

//    // use to access parameters from outside of C# and Unity
//    EnvironmentParameters m_ResetParams;
//    float force_magnitude = 0.0f;  // the force magnitude adds to each joint
//    int terminate_steps = 0;       // the max steps needs for each episode, use to calculate the time penalty

//    float target_distance = 0f;    // the target distance bewtween hand to holding location, 
//    float target_angle = 0f;       // the target angle distance between hand to up direction

//    // 0.0 means not add this constraint, 1.0 means add it
//    [HideInInspector]
//    public float add_target_angle = 0.0f; // use to check if need to add angle constraint(reward) into training

//    float add_dynamic_board = 0.0f; // use to check if need to add gravity constraint(reward) into training

//    [HideInInspector]
//    public float add_lifting_board = 0.0f; // use to check if need to add lifting board constraint(reward) into training.

//    //Vector3 board_target_pos;
//    float board_target_y; // the target position where the board supposed to stay at balance status

//    string log = ""; // use to hold the log information for check in exe window

//    Vector3 upper_arm_right_torque, lower_arm_right_torque, hand_right_torque, upper_arm_left_torque, lower_arm_left_torque, hand_left_torque;

//    float dist_ratio = 3f;
//    float palm_ratio = 2f;
//    float ang_ratio = 2f;

//    float r_reach = 0.001f;
//    float r_approch = 0.003f;

//    public override void Initialize()
//    {
//        woodyarea = GameObject.Find("WoodyArea").GetComponent<WoodyArea>();
//        woodyarea.AreaInit();

//        //woodysource = Resources.Load<GameObject>("Prefabs/woodybot");
//        woodysource = Resources.Load<GameObject>("Prefabs/woody_inuse7");

//        board = woodyarea.board;
//        target = woodyarea.target;
//        ball = woodyarea.ball;
//        boardrigid = board.gameObject.GetComponent<Rigidbody>();
//        ballrigid = ball.gameObject.GetComponent<Rigidbody>();

//        crosswin = woodyarea.crosswin;
//        crosslose = woodyarea.crosslose;

//        //board_target_pos = new Vector3(board.position.x, board.position.y + 0.2f, board.position.z);
//        board_target_y = board.position.y + 0.2f;
//        // each agent script will only need to focus on one agent's partial obersvation and action
//        // init agent's instance, its location and rotation, and all jionts rigidbody
//        BotBacktoOriginal(invertagent);

//        //agentfacedir = agent.transform.forward;

//        m_ResetParams = Academy.Instance.EnvironmentParameters;

//        SetResetParameters();

//        InitRewardCalculator();

//    }

//    // Update is called once per frame
//    public override void CollectObservations(VectorSensor sensor) // 18*6 + 10 + 7 = 251 dimension
//    {    
//        sensor.AddObservation(ballrigid.velocity);

//        sensor.AddObservation(target.position);
//        sensor.AddObservation(Mathf.Abs(board.position.y - board_target_y));

//        foreach (string tag in jointstag)
//        {
//            sensor.AddObservation(joints[tag].transform.position);
//            sensor.AddObservation(joints[tag].transform.rotation);

//            // here use to check the direction velocity relative to each side hold position, so need to seperate left and right
//            if (tag.Contains("left"))
//            {
//                sensor.AddObservation(lefthold.InverseTransformDirection(joints[tag].angularVelocity));
//                sensor.AddObservation(lefthold.InverseTransformDirection(joints[tag].velocity));
//            }

//            if (tag.Contains("right"))
//            {
//                sensor.AddObservation(righthold.InverseTransformDirection(joints[tag].angularVelocity));
//                sensor.AddObservation(righthold.InverseTransformDirection(joints[tag].velocity));
//            }
//        }

//        sensor.AddObservation(limbends["hand_left_end"].transform.rotation);
//        sensor.AddObservation(limbends["hand_right_end"].transform.rotation);

//        sensor.AddObservation(Vector3.Distance(joints["hand_left"].transform.position, lefthold.position));
//        sensor.AddObservation(Vector3.Distance(joints["hand_right"].transform.position, lefthold.position));

//    }

//    public override void OnActionReceived(ActionBuffers actionBuffers)
//    {
//        var vectoraction = actionBuffers.ContinuousActions;
//        float[] vectorAction = new float[action_count];
//        for (int i = 0; i < action_count; i++)
//        {
//            vectorAction[i] = vectoraction[i];
//        }
//        TakeAction(vectorAction);
//    }

//    public void TakeAction(float[] vectorAction)
//    {

//        float[] actions = new float[action_count];

//        for (int i = 0; i < action_count; i++)
//        {
//            actions[i] = Mathf.Clamp(vectorAction[i], -1, 1f) * force_magnitude;
//            //Debug.Log(i.ToString() + "th action= " + actions[i].ToString());
//        }

//        upper_arm_right_torque = new Vector3(actions[0], actions[1] / 100, actions[2]);
//        lower_arm_right_torque = new Vector3(actions[3], actions[4] / 100, actions[5]);
//        hand_right_torque = new Vector3(actions[6], actions[7] / 100, actions[8]);

//        upper_arm_left_torque = new Vector3(actions[9], actions[10] / 100, actions[11]);
//        lower_arm_left_torque = new Vector3(actions[12], actions[13] / 100, actions[14]);
//        hand_left_torque = new Vector3(actions[15], actions[16] / 100, actions[17]);

//        //joints["upper_arm_right"].AddForceAtPosition(upper_arm_right_torque, joints["lower_arm_right"].transform.position);
//        //joints["lower_arm_right"].AddForceAtPosition(lower_arm_right_torque, joints["hand_right"].transform.position);
//        //joints["hand_right"].AddForceAtPosition(hand_right_torque, limbends["hand_right_end"].position);

//        //joints["upper_arm_left"].AddForceAtPosition(upper_arm_left_torque, joints["lower_arm_right"].transform.position);
//        //joints["lower_arm_left"].AddForceAtPosition(lower_arm_left_torque, joints["hand_left"].transform.position);
//        //joints["hand_left"].AddForceAtPosition(hand_left_torque, limbends["hand_left_end"].position);

//        joints["upper_arm_right"].AddRelativeTorque(upper_arm_right_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
//        joints["lower_arm_right"].AddRelativeTorque(lower_arm_right_torque, ForceMode.Force); //, joints["hand_right"].transform.position);
//        joints["hand_right"].AddRelativeTorque(hand_right_torque, ForceMode.Force); //, limbends["hand_right_end"].position);

//        joints["upper_arm_left"].AddRelativeTorque(upper_arm_left_torque, ForceMode.Force); //, joints["lower_arm_right"].transform.position);
//        joints["lower_arm_left"].AddRelativeTorque(lower_arm_left_torque, ForceMode.Force); //, joints["hand_left"].transform.position);
//        joints["hand_left"].AddRelativeTorque(hand_left_torque, ForceMode.Force); //, limbends["hand_left_end"].position);

//        log = "";
//        countreach = 0;

//        // add reward when the hands reach to the target position
//        GetRewardHandDistance();

//        // add reward when the hands reach to the target angle
//        if (add_target_angle == 1.0)
//        {
//            GetRewardHandAngle();
//            GetRewardPalmTouchBoard();
//        }

//        // add reward when the board reaches to target position and target angle
//        if (add_lifting_board == 1.0)
//        {
//            GetRewardBoard();
//        }

//        // adding gravity of the board
//        if (add_dynamic_board == 1.0)
//        {
//            board.GetComponent<Rigidbody>().isKinematic = false;
//        }


//        // the reward to evaluate how close the ball to target
//        //GetRewardBallBalance();

//        // the reward to punish the time cost
//        //AddReward(-1 / terminate_steps);

//        //timer += Time.deltaTime;
//        //if (timer > 10f)
//        //{
//        //    TestActionFun(actions);
//        //    timer = 0;
//        //}


//        Debug.Log(this.name + ": " + log);

//        Debug.DrawRay(lefthold.position, Vector3.up * 0.5f, Color.red, Time.deltaTime, false);
//        Debug.DrawRay(righthold.position, Vector3.up * 0.5f, Color.red, Time.deltaTime, false);

//        Debug.DrawRay(limbends["hand_left_end"].position, -limbends["hand_left_end"].up * 0.5f, Color.green, Time.deltaTime, false);
//        Debug.DrawRay(limbends["hand_right_end"].position, -limbends["hand_right_end"].up * 0.5f, Color.green, Time.deltaTime, false);

//    }

//    public void GetRewardHandDistance()
//    {
//        // make the reward to get closer to the 4 corners of the hand
//        cur_leftdist = Vector3.Distance(lefthold.position, joints["hand_left"].transform.position);
//        cur_rightdist = Vector3.Distance(righthold.position, joints["hand_right"].transform.position);

//        if (cur_leftdist < target_distance)
//        {
//            AddReward(r_reach * dist_ratio);
//            countreach += 1;

//            log += "leftdist reached, ";
//        }
//        else
//        {
//            if (cur_leftdist > pre_leftdist) AddReward(-r_approch * dist_ratio);
//            else AddReward(r_approch * dist_ratio);

//            log += "leftdist= " + cur_leftdist.ToString("F3") + ", ";
//        }

//        if (cur_rightdist < target_distance)
//        {
//            AddReward(r_reach * dist_ratio);
//            countreach += 1;

//            log += "rightdist reached, ";
//        }
//        else
//        {
//            if (cur_rightdist > pre_rightdist) AddReward(-r_approch * dist_ratio);
//            else AddReward(r_approch * dist_ratio);

//            log += "rightdist= " + cur_rightdist.ToString("F3") + ", ";

//        }

//        pre_leftdist = cur_leftdist;
//        pre_rightdist = cur_rightdist;
//    }

//    public void GetRewardHandAngle()
//    {
//        //cur_leftang = Vector3.Angle( -limbends["hand_left_end"].right, -lefthold.up);
//        //cur_rightang = Vector3.Angle(-limbends["hand_right_end"].right, -righthold.up);

//        cur_leftang = Vector3.Angle(-limbends["hand_left_end"].up, Vector3.up);
//        cur_rightang = Vector3.Angle(-limbends["hand_right_end"].up, Vector3.up);

//        if (cur_leftang < target_angle)
//        {
//            AddReward(r_reach * ang_ratio);
//            countreach += 1;

//            log += "leftangle reached, ";
//        }
//        else
//        {
//            if (cur_leftang > pre_leftang) AddReward(-r_approch * ang_ratio);
//            else AddReward(r_approch * ang_ratio);

//            log += "leftangle= " + cur_leftang.ToString("F1") + ", ";

//        }

//        if (cur_rightang < target_angle)
//        {
//            AddReward(r_reach * ang_ratio);
//            countreach += 1;

//            log += "rightangle reached, ";
//        }
//        else
//        {
//            if (cur_rightang > pre_rightang) AddReward(-r_approch * ang_ratio);
//            else AddReward(r_approch * ang_ratio);

//            log += "rightangle= " + cur_rightang.ToString("F1") + ", ";

//        }

//        pre_leftang = cur_leftang;
//        pre_rightang = cur_rightang;
//    }

//    public void GetRewardPalmTouchBoard()
//    {
//        if (Physics.Raycast(limbends["hand_left_end"].position, limbends["hand_left_end"].TransformDirection(-Vector3.up), out lefthit, Mathf.Infinity) && lefthit.collider.tag == "tray")
//        {
//            AddReward(r_reach * palm_ratio);
//            countreach += 1;

//            log += "leftpalm reached, ";
//        }
//        else
//        {
//            AddReward(-r_reach * palm_ratio);
//            log += "leftpalm not reach, ";
//        }

//        if (Physics.Raycast(limbends["hand_right_end"].position, limbends["hand_right_end"].TransformDirection(-Vector3.up), out righthit, Mathf.Infinity) && righthit.collider.tag == "tray")
//        {
//            AddReward(r_reach * palm_ratio);
//            countreach += 1;

//            log += "rightpalm reached, ";
//        }
//        else
//        {
//            AddReward(-r_reach * palm_ratio);
//            log += "rightpalm not reach, ";
//        }

//    }

//    public void GetRewardBoard()
//    {
//        cur_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
//        cur_boardang = Vector3.Angle(board.up, Vector3.up);

//        if (cur_boardang < target_angle)
//        {
//            AddReward(r_reach);
//            countreach += 1;

//            log += "boardangle reached, ";
//        }
//        else
//        {
//            if (cur_boardang > pre_boardang) AddReward(-r_approch);
//            else AddReward(r_approch);

//            log += "boardangle= " + cur_boardang.ToString("F1") + ", ";
//        }
//        pre_boardang = cur_boardang;

//        if (cur_boarddist_y < 0.1f)
//        {
//            AddReward(r_reach);
//            countreach += 1;

//            log += "boarddist reached, ";
//        }
//        else
//        {
//            if (cur_boarddist_y > pre_boarddist_y) AddReward(-r_approch);
//            else AddReward(r_approch);

//            log += "boarddist= " + cur_boarddist_y.ToString("F3") + ", ";
//        }

//        pre_boarddist_y = cur_boarddist_y;


//        /*
//         // add reward to compare the normal direction of board and ground
//        if (add_lifting_board == 1.0)
//        {
//            float normdiff = Vector3.Distance(board.up, Vector3.up); // range[0, 1.414]
//            float r_normdiff = (0.5f - normdiff) * 0.1f;

//            if (board.position.y > 1.1f)
//            {
//                //if (normdiff > 0.8f)
//                //{
//                //    AddReward(-0.05f);
//                //    checkdone = true;
//                //    //EndEpisode();
//                //}
//                //else
//                //{
//                //    AddReward(r_normdiff);
//                //}
//                AddReward(r_normdiff);
//            }
//        }
//         */
//    }

//    public void GetRewardBallBalance()
//    {
//        cur_balldist = Vector3.Distance(target.position, ball.position);

//        //r_dist = -(curdist - predist) / woodyarea.board_length;
//        //AddReward(r_dist);

//        if (cur_balldist < target_distance * 0.5f)
//        {
//            AddReward(r_reach);
//            countreach += 1;

//            log += "ball reached, ";
//        }
//        else
//        {
//            if (cur_balldist > pre_balldist) AddReward(-r_approch);
//            else AddReward(r_approch);

//            log += "balldist= " + cur_balldist.ToString("F3") + ", ";
//        }
//        pre_balldist = cur_balldist;
//    }

//    public override void Heuristic(in ActionBuffers actionsOut)
//    {

//        float torqueforce = 10f;
//        var actions = actionsOut.ContinuousActions;
//        float[] action = new float[action_count];

//        for (int i = 0; i < action_count; i++)
//        {
//            action[i] = actions[i];
//        }

//        if (Input.GetKey(KeyCode.Q)) actions[0] = torqueforce;

//        if (Input.GetKey(KeyCode.A)) actions[0] = -torqueforce;

//        if (Input.GetKey(KeyCode.W)) actions[1] = torqueforce;

//        if (Input.GetKey(KeyCode.S)) actions[1] = -torqueforce;

//        if (Input.GetKey(KeyCode.E)) actions[2] = torqueforce;

//        if (Input.GetKey(KeyCode.D)) actions[2] = -torqueforce;

//        if (Input.GetKey(KeyCode.R)) actions[3] = torqueforce;

//        if (Input.GetKey(KeyCode.F)) actions[3] = -torqueforce;

//        if (Input.GetKey(KeyCode.T)) actions[4] = torqueforce;

//        if (Input.GetKey(KeyCode.G)) actions[4] = -torqueforce;

//        if (Input.GetKey(KeyCode.Y)) actions[5] = torqueforce;

//        if (Input.GetKey(KeyCode.H)) actions[5] = -torqueforce;

//        if (Input.GetKey(KeyCode.U)) actions[6] = torqueforce;

//        if (Input.GetKey(KeyCode.J)) actions[6] = -torqueforce;

//        if (Input.GetKey(KeyCode.I)) actions[7] = torqueforce;

//        if (Input.GetKey(KeyCode.K)) actions[7] = -torqueforce;

//        if (Input.GetKey(KeyCode.O)) actions[8] = torqueforce;

//        if (Input.GetKey(KeyCode.L)) actions[8] = -torqueforce;

//        if (Input.GetKey(KeyCode.Z)) actions[9] = torqueforce;

//        if (Input.GetKey(KeyCode.X)) actions[9] = -torqueforce;

//        if (Input.GetKey(KeyCode.C)) actions[10] = torqueforce;

//        if (Input.GetKey(KeyCode.V)) actions[10] = -torqueforce;

//        if (Input.GetKey(KeyCode.B)) actions[11] = torqueforce;

//        if (Input.GetKey(KeyCode.N)) actions[11] = -torqueforce;

//        if (Input.GetKey(KeyCode.M)) actions[12] = torqueforce;

//        if (Input.GetKey(KeyCode.P)) actions[12] = -torqueforce;

//        if (Input.GetKey(KeyCode.LeftArrow)) actions[13] = torqueforce;

//        if (Input.GetKey(KeyCode.RightArrow)) actions[13] = -torqueforce;


//        //TestActionFun(action);
//    }

//    public void TestActionFun(float[] action)
//    {
//        if (action[0] > 0) Debug.Log(this.name + " Q pressed, upper_arm_right X joint force increased" + " " + action[0] + " " + joints["upper_arm_right"].transform.eulerAngles);

//        if (action[0] < 0) Debug.Log(this.name + " A pressed, upper_arm_right X joint force decreased" + " " + action[0] + " " + joints["upper_arm_right"].transform.eulerAngles);

//        if (action[1] > 0) Debug.Log(this.name + " W pressed, upper_arm_right Y joint force increased" + " " + action[1] + " " + joints["upper_arm_right"].transform.eulerAngles);

//        if (action[1] < 0) Debug.Log(this.name + " S pressed, upper_arm_right Y joint force decreased" + " " + action[1] + " " + joints["upper_arm_right"].transform.eulerAngles);

//        if (action[2] > 0) Debug.Log(this.name + " E pressed, upper_arm_right Z joint force increased" + " " + action[2] + " " + joints["upper_arm_right"].transform.eulerAngles);

//        if (action[2] < 0) Debug.Log(this.name + " D pressed, upper_arm_right Z joint force decreased" + " " + action[2] + " " + joints["upper_arm_right"].transform.eulerAngles);

//        if (action[3] > 0) Debug.Log(this.name + " R pressed, lower_arm_right X joint force increased" + " " + action[3] + " " + joints["lower_arm_right"].transform.eulerAngles);

//        if (action[3] < 0) Debug.Log(this.name + " F pressed, lower_arm_right X joint force decreased" + " " + action[3] + " " + joints["lower_arm_right"].transform.eulerAngles);

//        if (action[4] > 0) Debug.Log(this.name + " T pressed, lower_arm_right Z joint force increased" + " " + action[4] + " " + joints["lower_arm_right"].transform.eulerAngles);

//        if (action[4] < 0) Debug.Log(this.name + " G pressed, lower_arm_right Z joint force decreased" + " " + action[4] + " " + joints["lower_arm_right"].transform.eulerAngles);

//        if (action[5] > 0) Debug.Log(this.name + " Y pressed, hand_right X joint force increased" + " " + action[5] + " " + joints["hand_right"].transform.eulerAngles);

//        if (action[5] < 0) Debug.Log(this.name + " H pressed, hand_right X joint force decreased" + " " + action[5] + " " + joints["hand_right"].transform.eulerAngles);

//        if (action[6] > 0) Debug.Log(this.name + " U pressed, hand_right Z joint force increased" + " " + action[6] + " " + joints["hand_right"].transform.eulerAngles);

//        if (action[6] < 0) Debug.Log(this.name + " J pressed, hand_right Z joint force decreased" + " " + action[6] + " " + joints["hand_right"].transform.eulerAngles);

//        if (action[7] > 0) Debug.Log(this.name + " I pressed, upper_arm_left X joint force increased" + " " + action[7] + " " + joints["upper_arm_left"].transform.eulerAngles);

//        if (action[7] < 0) Debug.Log(this.name + " K pressed, upper_arm_left X joint force decreased" + " " + action[7] + " " + joints["upper_arm_left"].transform.eulerAngles);

//        if (action[8] > 0) Debug.Log(this.name + " O pressed, upper_arm_left Y joint force increased" + " " + action[8] + " " + joints["upper_arm_left"].transform.eulerAngles);

//        if (action[8] < 0) Debug.Log(this.name + " L pressed, upper_arm_left Y joint force decreased" + " " + action[8] + " " + joints["upper_arm_left"].transform.eulerAngles);

//        if (action[9] > 0) Debug.Log(this.name + " Z pressed, upper_arm_left Z joint force increased" + " " + action[9] + " " + joints["upper_arm_left"].transform.eulerAngles);

//        if (action[9] < 0) Debug.Log(this.name + " X pressed, upper_arm_left Z joint force decreased" + " " + action[9] + " " + joints["upper_arm_left"].transform.eulerAngles);

//        if (action[10] > 0) Debug.Log(this.name + " C pressed, lower_arm_left X joint force increased" + " " + action[10] + " " + joints["lower_arm_left"].transform.eulerAngles);

//        if (action[10] < 0) Debug.Log(this.name + " V pressed, lower_arm_left X joint force decreased" + " " + action[10] + " " + joints["lower_arm_left"].transform.eulerAngles);

//        if (action[11] > 0) Debug.Log(this.name + " B pressed, lower_arm_left Z joint force increased" + " " + action[11] + " " + joints["lower_arm_left"].transform.eulerAngles);

//        if (action[11] < 0) Debug.Log(this.name + " N pressed, lower_arm_left Z joint force decreased" + " " + action[11] + " " + joints["lower_arm_left"].transform.eulerAngles);

//        if (action[12] > 0) Debug.Log(this.name + " M pressed, hand_left X joint force increased" + " " + action[12] + " " + joints["hand_left"].transform.eulerAngles);

//        if (action[12] < 0) Debug.Log(this.name + " P pressed, hand_left X joint force decreased" + " " + action[12] + " " + joints["hand_left"].transform.eulerAngles);

//        if (action[13] > 0) Debug.Log(this.name + " leftarrow pressed, hand_left Z joint force increased" + " " + action[13] + " " + joints["hand_left"].transform.eulerAngles);

//        if (action[13] < 0) Debug.Log(this.name + " rightarrow pressed, hand_left Z joint force decreased" + " " + action[13] + " " + joints["hand_left"].transform.eulerAngles);

//    }

//    public override void OnEpisodeBegin()
//    {

//        woodyarea.AreaReset();
//        BotBacktoOriginal(invertagent);
//        //reset the force ratio here as a accessible paramters in python api
//        SetResetParameters();
//        InitRewardCalculator();

//    }

//    void SetResetParameters()
//    {
//        force_magnitude = m_ResetParams.GetWithDefault("force_magnitude", 500.0f);
//        terminate_steps = (int)m_ResetParams.GetWithDefault("terminate_steps", 3000f);

//        target_distance = m_ResetParams.GetWithDefault("target_distance", 0.02f);
//        target_angle = m_ResetParams.GetWithDefault("target_angle", 5f);

//        add_dynamic_board = m_ResetParams.GetWithDefault("add_dynamic_board", 0.0f);
//        add_lifting_board = m_ResetParams.GetWithDefault("add_lifting_board", 0.0f);
//        add_target_angle = m_ResetParams.GetWithDefault("add_target_angle", 0.0f);

//        checkdone = false;
//    }

//    void InitRewardCalculator()
//    {
//        //pre_leftdist = Vector3.Distance(lefthold.position, limbends["hand_left_end"].position);
//        //pre_rightdist = Vector3.Distance(righthold.position, limbends["hand_right_end"].position);

//        pre_leftdist = Vector3.Distance(lefthold.position, joints["hand_left"].transform.position);
//        pre_rightdist = Vector3.Distance(righthold.position, joints["hand_right"].transform.position);

//        //pre_leftang = Vector3.Angle(-limbends["hand_left_end"].right, -lefthold.up);
//        //pre_rightang = Vector3.Angle(-limbends["hand_right_end"].right, -righthold.up);

//        pre_leftang = Vector3.Angle(-limbends["hand_left_end"].up, Vector3.up);
//        pre_rightang = Vector3.Angle(-limbends["hand_right_end"].up, Vector3.up);

//        pre_boarddist_y = Mathf.Abs(board.position.y - board_target_y);
//        pre_boardang = Vector3.Angle(board.up, Vector3.up);

//        pre_balldist = Vector3.Distance(target.position, ball.position);

//    }

//    //-----------------------------------------------------------------------------
//    public void BotBacktoOriginal(bool invertagent)
//    {
//        // remove the current agent in the scene
//        if (agent != null)
//        {
//            //joints["hand_right"].transform.parent = null;
//            //joints["hand_left"].transform.parent = null;
//            Destroy(agent);
//        }
//        // instantiate the agent again
//        // each agent script will only need to focus on one agent's partial obersvation and action
//        if (invertagent)
//        {
//            float x1 = woodyarea.pos_agent1.x;
//            float y1 = woodyarea.pos_agent1.y;
//            float z1 = woodyarea.pos_agent1.z;

//            float rx1 = woodyarea.rotang_agent1.x;
//            float ry1 = woodyarea.rotang_agent1.y;
//            float rz1 = woodyarea.rotang_agent1.z;

//            agent = Instantiate(woodysource, new Vector3(x1, y1, z1), Quaternion.Euler(rx1, ry1, rz1));
//            agent.transform.parent = woodyarea.box_agent1;

//        }
//        else
//        {
//            float x2 = woodyarea.pos_agent2.x;
//            float y2 = woodyarea.pos_agent2.y;
//            float z2 = woodyarea.pos_agent2.z;

//            float rx2 = woodyarea.rotang_agent2.x;
//            float ry2 = woodyarea.rotang_agent2.y;
//            float rz2 = woodyarea.rotang_agent2.z;

//            agent = Instantiate(woodysource, new Vector3(x2, y2, z2), Quaternion.Euler(rx2, ry2, rz2));
//            agent.transform.parent = woodyarea.box_agent2;
//        }

//        // setup all joints info
//        jointstag.Clear();
//        jointstag.Add("upper_arm_right");
//        jointstag.Add("lower_arm_right");
//        jointstag.Add("hand_right");

//        jointstag.Add("upper_arm_left");
//        jointstag.Add("lower_arm_left");
//        jointstag.Add("hand_left");

//        joints.Clear();
//        foreach (string tag in jointstag)
//        {
//            FindChildObjectwithTag(agent.transform, tag); // store all the joints into the dictionary
//        }

//        limbends.Clear();
//        FindChildObjectwithTag_Transform(agent.transform, "hand_right_end");
//        FindChildObjectwithTag_Transform(agent.transform, "hand_left_end");

//    }

//    public void FindChildObjectwithTag(Transform parent, string _tag)
//    {
//        for (int i = 0; i < parent.childCount; i++)
//        {
//            Transform child = parent.GetChild(i);
//            if (child.tag == _tag)
//            {
//                joints.Add(_tag, child.GetComponent<Rigidbody>());
//                break;
//            }
//            if (child.childCount > 0)
//            {
//                FindChildObjectwithTag(child, _tag);
//            }
//        }
//    }

//    public void FindChildObjectwithTag_Transform(Transform parent, string _tag)
//    {
//        for (int i = 0; i < parent.childCount; i++)
//        {
//            Transform child = parent.GetChild(i);
//            if (child.tag == _tag)
//            {
//                limbends.Add(_tag, child);
//                break;
//            }
//            if (child.childCount > 0)
//            {
//                FindChildObjectwithTag_Transform(child, _tag);
//            }
//        }
//    }
//}

