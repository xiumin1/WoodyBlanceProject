//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;
//using MLAgents;

//public class WoodyAgent01 : Agent
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
//    //public List<GameObject> joints = new List<GameObject>(); 
//    private List<string> jointstag = new List<string>();

//    int action_count = 14;

//    Material crosswin, crosslose;

//    private Vector3 agentfacedir; // the facing directioin of the current agent

//    int maxstep = 2000;

//    float predist, curdist, r_dist;

//    // the following 4 lines define the hands moving to the board to hold it
//    Transform a1L, a1R, a2L, a2R; // the 4 joints on board
//    Transform ltargetjoint, rtargetjoint, rstart, lstart; // the target joints of the current agent

//    float startTime, finishedJourney, speed = 1.0f;
//    float rjourneyLength, rfracJourney, ljourneyLength, lfracJourney;

//    Rigidbody ltargetjointrigid, rtargetjointrigid;
//    bool connectjoint = false; // use to make sure when to connect the hand and board

//    Transform rfixjoint1, lfixjoint1, rfixjoint2, lfixjoint2;
//    Transform rfixjoint, lfixjoint;
//    public override void InitializeAgent()
//    {

//        woodyarea = GameObject.Find("WoodyArea").GetComponent<WoodyArea>();
//        woodyarea.AreaInit();

//        //woodysource = Resources.Load<GameObject>("Prefabs/woodybot");
//        woodysource = Resources.Load<GameObject>("Prefabs/woodcharhingejoint");

//        board = woodyarea.board;
//        target = woodyarea.target;
//        ball = woodyarea.ball;
//        boardrigid = board.gameObject.GetComponent<Rigidbody>();
//        ballrigid = ball.gameObject.GetComponent<Rigidbody>();

        
//        a1L = woodyarea.a1L;
//        a1R = woodyarea.a1R;
//        a2L = woodyarea.a2L;
//        a2R = woodyarea.a2R;

//        rfixjoint1 = a1L;
//        lfixjoint1 = a1R;
//        rfixjoint2 = a2L;
//        lfixjoint2 = a2R;

//        crosswin = woodyarea.crosswin;
//        crosslose = woodyarea.crosslose;
//        // each agent script will only need to focus on one agent's partial obersvation and action
//        // init agent's instance, its location and rotation, and all jionts rigidbody
//        BotBacktoOriginal(invertagent);

//        agentfacedir = agent.transform.forward;

//        predist = Vector3.Distance(target.position, ball.position);
//    }

//    // Update is called once per frame
//    public override void CollectObservations()
//    {
//        AddVectorObs(board.position);
//        AddVectorObs(ball.position);
//        AddVectorObs(target.position);

//        foreach (string tag in jointstag)
//        {
//            AddVectorObs(joints[tag].transform.rotation.eulerAngles);
//        }

//    }

//    public override void AgentAction(float[] vectorAction)
//    {
//        float[] reachfracs = ReachFracs();
//        //reachfracs = ReachFracs();
//        if (reachfracs[0] < 1.0f || reachfracs[1] < 1.0f)
//        {
//            ReachBoard();
//        }
//        else
//        {
//            joints["hand_right"].transform.position = rtargetjoint.position;
//            joints["hand_left"].transform.position = ltargetjoint.position;

//            joints["hand_right"].transform.rotation = rtargetjoint.rotation;
//            joints["hand_left"].transform.rotation = ltargetjoint.rotation;

//            if (!connectjoint)
//            {
//                //HingeJoint rjoint;
//                //rtargetjoint.gameObject.AddComponent<HingeJoint>();
//                //rjoint = rtargetjoint.GetComponent<HingeJoint>();
//                //rjoint.connectedBody = joints["hand_right"];

//                //HingeJoint ljoint;
//                //ltargetjoint.gameObject.AddComponent<HingeJoint>();
//                //ljoint = ltargetjoint.GetComponent<HingeJoint>();
//                //ljoint.connectedBody = joints["hand_left"];

//                //boardrigid.usegravity = true;
//                boardrigid.isKinematic = false;
//                ballrigid.isKinematic = false;
                

//                //ltargetjointrigid.isKinematic = false;
//                //rtargetjointrigid.isKinematic = false;

//                //joints["hand_right"].transform.parent = rtargetjoint;
//                //joints["hand_left"].transform.parent = ltargetjoint;

//                connectjoint = true;
//            }
          
//        }

//        //// make sure the target joint will always follow its original position on the board, 
//        //// because target joint is a rigidbody and will change its position because of physics force added from hand
//        //ltargetjoint.position = lfixjoint.position;
//        //rtargetjoint.position = rfixjoint.position;

//        //ltargetjoint.rotation = lfixjoint.rotation;
//        //rtargetjoint.rotation = rfixjoint.rotation;

//        // if the hands connect to board, then start the RL actions 
//        if (connectjoint)
//        {
//            BalanceAction(vectorAction);
//        }
//    }

//    public void BalanceAction(float[] vectorAction)
//    {
//        float[] actions = new float[action_count];

//        for (int i = 0; i < action_count; i++)
//        {
//            actions[i] = Mathf.Clamp(vectorAction[i], -1, 1f) * 100f;
//            //Debug.Log(i + " actions value= " + actions[i]);
//        }

//        //actions[0] = -10f;
//        //actions[3] = -10f;
//        //actions[7] = 0.5f;
//        Vector3 upper_arm_right_torque = new Vector3(actions[0], actions[1], actions[2]);
//        Vector3 lower_arm_right_torque = new Vector3(actions[3], 0, actions[4]);
//        Vector3 hand_right_torque = new Vector3(actions[5], 0, actions[6]);

//        Vector3 upper_arm_left_torque = new Vector3(actions[7], actions[8], actions[9]);
//        Vector3 lower_arm_left_torque = new Vector3(actions[10], 0, actions[11]);
//        Vector3 hand_left_torque = new Vector3(actions[12], 0, actions[13]);

//        //joints["upper_arm_right"].AddTorque(upper_arm_right_torque, ForceMode.Force);
//        //joints["lower_arm_right"].AddTorque(lower_arm_right_torque, ForceMode.Force);
//        //joints["hand_right"].AddTorque(hand_right_torque, ForceMode.Force);

//        //joints["upper_arm_left"].AddTorque(upper_arm_left_torque, ForceMode.Force);
//        //joints["lower_arm_left"].AddTorque(lower_arm_left_torque, ForceMode.Force);
//        //joints["hand_left"].AddTorque(hand_left_torque, ForceMode.Force);

//        joints["upper_arm_right"].AddForceAtPosition(upper_arm_right_torque, joints["lower_arm_right"].transform.position);
//        joints["lower_arm_right"].AddForceAtPosition(lower_arm_right_torque, joints["hand_right"].transform.position);
//        joints["hand_right"].AddForceAtPosition(hand_right_torque, limbends["hand_right_end"].position);

//        joints["upper_arm_left"].AddForceAtPosition(upper_arm_left_torque, joints["lower_arm_right"].transform.position);
//        joints["lower_arm_left"].AddForceAtPosition(lower_arm_left_torque, joints["hand_left"].transform.position);
//        joints["hand_left"].AddForceAtPosition(hand_left_torque, limbends["hand_left_end"].position);


//        curdist = Vector3.Distance(target.position, ball.position);

//        // the reward to evaluate how close the ball to target
//        r_dist = -(curdist - predist) / woodyarea.board_length;
//        AddReward(r_dist);

//        // the reward to punish the time cost
//        AddReward(-1 / maxstep);

//        //Debug.Log("the current cumulated reward is: " + GetReward());

//        if (woodyarea.board.position.y < 0.3f)
//        {
//            AddReward(-1f);
//            Done();
//            Debug.Log("may not need this check for board dropping on the floor");
//        }
//    }
//    public override float[] Heuristic()
//    {
//        float[] actions = new float[14];

//        if (Input.GetKey(KeyCode.Q))
//        {
//            actions[0] = 0.1f;
//        }
//        if (Input.GetKey(KeyCode.A))
//        {
//            actions[0] = -0.1f;
//        }
//        if (Input.GetKey(KeyCode.W))
//        {
//            actions[1] = 0.1f;
//        }

//        if (Input.GetKey(KeyCode.S))
//        {
//            actions[1] = -0.1f;
//        }
//        if (Input.GetKey(KeyCode.E))
//        {
//            actions[2] = 0.1f;
//        }
//        if (Input.GetKey(KeyCode.D))
//        {
//            actions[2] = -0.1f;
//        }

//        if (Input.GetKey(KeyCode.R))
//        {
//            actions[3] = .1f;
//        }
//        if (Input.GetKey(KeyCode.F))
//        {
//            actions[3] = .1f;
//        }
//        if (Input.GetKey(KeyCode.T))
//        {
//            actions[4] = .1f;
//        }

//        if (Input.GetKey(KeyCode.G))
//        {
//            actions[4] = -.1f;
//        }
//        if (Input.GetKey(KeyCode.Y))
//        {
//            actions[5] = .1f;
//        }
//        if (Input.GetKey(KeyCode.H))
//        {
//            actions[5] = -.1f;
//        }

//        if (Input.GetKey(KeyCode.U))
//        {
//            actions[6] = .1f;
//        }
//        if (Input.GetKey(KeyCode.J))
//        {
//            actions[6] = -.1f;
//        }
//        if (Input.GetKey(KeyCode.I))
//        {
//            actions[7] = .1f;
//        }
//        if (Input.GetKey(KeyCode.K))
//        {
//            actions[7] = -.1f;
//        }
//        if (Input.GetKey(KeyCode.O))
//        {
//            actions[8] = .1f;
//        }
//        if (Input.GetKey(KeyCode.L))
//        {
//            actions[8] = -.1f;
//        }

//        return actions;
//    }
//    public override void AgentReset()
//    {
//        woodyarea.AreaReset();
//        BotBacktoOriginal(invertagent);
//    }

//    public static Vector3 GetGlobalToLocalScaleFactor(Transform t, Vector3 scale)
//    {
//        Vector3 factor = Vector3.one;

//        while (true)
//        {
//            Transform tParent = t.parent;

//            if (tParent != null)
//            {
//                factor.x *= tParent.localScale.x;
//                factor.y *= tParent.localScale.y;
//                factor.z *= tParent.localScale.z;

//                t = tParent;
//            }
//            else
//            {
//                break;
//                //return factor;
//            }
//        }

//        Vector3 newlocalscale = new Vector3(scale.x / factor.x, scale.y / factor.y, scale.z / factor.z);

//        return newlocalscale;
//    }

//    private float SmallRotateAngle(float ang1, float ang2)
//    {
//        float angle = 0;
//        angle = Mathf.Abs(ang1 - ang2);

//        if (angle > 90f && angle < 180f)
//        {
//            angle = 180f - angle;
//        }
//        if (angle > 180f && angle < 270f)
//        {
//            angle = angle - 180f;
//        }
//        if (angle > 270f && angle < 360f)
//        {
//            angle = 360f - angle;
//        }

//        return angle;
//    }

//    IEnumerator waitidle(float t)
//    {
//        yield return new WaitForSeconds(t);
//    }


//    //-----------------------------------------------------------------------------
//    public void BotBacktoOriginal(bool invertagent)
//    {
//        //setup the board and ball physics info to make it stay in the air
//        //boardrigid.useGravity = false;
//        boardrigid.isKinematic = true;

//        //ballrigid.useGravity = false;
//        ballrigid.isKinematic = true;

//        connectjoint = false;
//        // remove the current agent in the scene
//        if (agent != null)
//        {
//            joints["hand_right"].transform.parent = null;
//            joints["hand_left"].transform.parent = null;

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

//        // setup the hand movement info
//        ReachBoardInit(invertagent);
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

//    public void ReachBoardInit(bool invertagent) // get the starting time and init distance from the hand to target joint location
//    {
//        if (invertagent)
//        {
//            ltargetjoint = a1L;
//            rtargetjoint = a1R;

//            //lfixjoint = lfixjoint1;
//            //rfixjoint = rfixjoint1;
//        }
//        else
//        {
//            ltargetjoint = a2L;
//            rtargetjoint = a2R;

//            //lfixjoint = lfixjoint2;
//            //rfixjoint = rfixjoint2;
//        }

//        //lfixjoint.parent = board.transform;
//        //rfixjoint.parent = board.transform;

//        //ltargetjoint.position = lfixjoint.position;
//        //ltargetjoint.rotation = lfixjoint.rotation;

//        //rtargetjoint.position = rfixjoint.position;
//        //rtargetjoint.rotation = rtargetjoint.rotation;

//        //ltargetjointrigid = ltargetjoint.GetComponent<Rigidbody>();
//        //rtargetjointrigid = rtargetjoint.GetComponent<Rigidbody>();

//        //ltargetjointrigid.isKinematic = true;
//        //rtargetjointrigid.isKinematic = true;
        
//        startTime = Time.time;

//        rstart = joints["hand_right"].transform;
//        lstart = joints["hand_left"].transform;

//        rjourneyLength = Vector3.Distance(rstart.position, rtargetjoint.position);
//        ljourneyLength = Vector3.Distance(lstart.position, ltargetjoint.position);
//    }

//    public float[] ReachFracs() // calculate the fraction where the current hand is, how long the hand has moved along the journey
//    {
//        float[] fracs = new float[2];
//        finishedJourney = (Time.time - startTime) * speed;

//        rfracJourney = finishedJourney / rjourneyLength;
//        lfracJourney = finishedJourney / ljourneyLength;

//        //rfracJourney = -2 * Mathf.Pow(rfracJourney, 3.0f) + 3 * Mathf.Pow(rfracJourney, 2.0f);//
//        //lfracJourney = -2 * Mathf.Pow(lfracJourney, 3.0f) + 3 * Mathf.Pow(lfracJourney, 2.0f);//

//        fracs[0] = rfracJourney;
//        fracs[1] = lfracJourney;

//        return fracs;
//    }

//    public void ReachBoard()
//    {

//        joints["hand_right"].transform.position = Vector3.Slerp(rstart.position, rtargetjoint.position, rfracJourney);
//        joints["hand_left"].transform.position = Vector3.Slerp(lstart.position, ltargetjoint.position, lfracJourney);

//        joints["hand_right"].transform.rotation = Quaternion.Slerp(rstart.rotation, rtargetjoint.rotation, rfracJourney);
//        joints["hand_left"].transform.rotation = Quaternion.Slerp(lstart.rotation, ltargetjoint.rotation, lfracJourney);
//    }

//}
