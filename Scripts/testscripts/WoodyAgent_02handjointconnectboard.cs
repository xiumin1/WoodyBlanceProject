//using system.collections;
//using system.collections.generic;
//using unityengine;
//using unityengine.ui;
//using unity.mlagents;
//using unity.mlagents.actuators;
//using unity.mlagents.sensors;

//public class woodyagent : agent
//{
//    //------------------------------------------------------------------------------------------
//    // the two agents init position and the whole environment info will be initilized and reset in the woodyarea script for simplicity
//    public bool invertagent; // this bool value will help to differenciate the two agent

//    gameobject woodysource, agent; // source prefab and the generated agent

//    woodyarea woodyarea;
//    transform board, target, ball;
//    rigidbody boardrigid, ballrigid;

//    private dictionary<string, rigidbody> joints = new dictionary<string, rigidbody>(); // to store the 6 joints of two arms

//    private dictionary<string, transform> limbends = new dictionary<string, transform>();
//    //public list<gameobject> joints = new list<gameobject>(); 
//    private list<string> jointstag = new list<string>();

//    int action_count = 14;

//    material crosswin, crosslose;

//    private vector3 agentfacedir; // the facing directioin of the current agent

//    int maxstep = 2000;

//    float predist, curdist, r_dist;

//    // the following 4 lines define the hands moving to the board to hold it
//    transform a1l, a1r, a2l, a2r; // the 4 joints on board
//    transform ltargetjoint, rtargetjoint, rstart, lstart; // the target joints of the current agent

//    float starttime, finishedjourney, speed = 1.0f;
//    float rjourneylength, rfracjourney, ljourneylength, lfracjourney;

//    bool connectjoint = false; // use to make sure when to connect the hand and board

//    public override void initialize()
//    {
//        debug.log("initialize called!");

//        woodyarea = gameobject.find("woodyarea").getcomponent<woodyarea>();
//        woodyarea.areainit();

//        //woodysource = resources.load<gameobject>("prefabs/woodybot");
//        woodysource = resources.load<gameobject>("prefabs/woodcharhingejoint");

//        board = woodyarea.board;
//        target = woodyarea.target;
//        ball = woodyarea.ball;
//        boardrigid = board.gameobject.getcomponent<rigidbody>();
//        ballrigid = ball.gameobject.getcomponent<rigidbody>();

//        a1l = woodyarea.a1l;
//        a1r = woodyarea.a1r;
//        a2l = woodyarea.a2l;
//        a2r = woodyarea.a2r;

//        crosswin = woodyarea.crosswin;
//        crosslose = woodyarea.crosslose;
//        // each agent script will only need to focus on one agent's partial obersvation and action
//        // init agent's instance, its location and rotation, and all jionts rigidbody
//        botbacktooriginal(invertagent);

//        agentfacedir = agent.transform.forward;

//        predist = vector3.distance(target.position, ball.position);
//    }

//    // update is called once per frame
//    public override void collectobservations(vectorsensor sensor)
//    {
//        debug.log("collectobservations called!");

//        sensor.addobservation(board.position);
//        sensor.addobservation(ball.position);
//        sensor.addobservation(target.position);

//        foreach (string tag in jointstag)
//        {
//            sensor.addobservation(joints[tag].transform.rotation.eulerangles);
//        }

//    }

//    public override void onactionreceived(actionbuffers actionbuffers)
//    {
//        debug.log("onactionreceived called!");
//        var vectoraction = actionbuffers.continuousactions;
//        float[] vectoraction = new float[action_count];
//        for (int i = 0; i < action_count; i++)
//        {
//            vectoraction[i] = vectoraction[i];
//            debug.log(i + " actions value= " + vectoraction[i]);
//        }
//        float[] reachfracs = reachfracs();
//        //reachfracs = reachfracs();
//        if (reachfracs[0] < 1.0f || reachfracs[1] < 1.0f)
//        {
//            reachboard();
//        }
//        else
//        {
//            joints["hand_right"].transform.position = rtargetjoint.position;
//            joints["hand_left"].transform.position = ltargetjoint.position;

//            joints["hand_right"].transform.rotation = rtargetjoint.rotation;
//            joints["hand_left"].transform.rotation = ltargetjoint.rotation;

//            if (!connectjoint)
//            {
//                boardrigid.iskinematic = false;
//                ballrigid.iskinematic = false;
//                connectjoint = true;
//            }
//        }

//        //// make sure the target joint will always follow its original position on the board, 
//        //// because target joint is a rigidbody and will change its position because of physics force added from hand
//        //ltargetjoint.position = lfixjoint.position;
//        //rtargetjoint.position = rfixjoint.position;

//        //ltargetjoint.rotation = lfixjoint.rotation;
//        //rtargetjoint.rotation = rfixjoint.rotation;

//        // if the hands connect to board, then start the rl actions 
//        if (connectjoint)
//        {
//            balanceaction(vectoraction);
//        }
//    }

//    public void balanceaction(float[] vectoraction)
//    {
//        float[] actions = new float[action_count];

//        for (int i = 0; i < action_count; i++)
//        {
//            actions[i] = mathf.clamp(vectoraction[i], -1, 1f) * 100f;

//        }

//        //actions[0] = -10f;
//        //actions[3] = -10f;
//        //actions[7] = 0.5f;
//        vector3 upper_arm_right_torque = new vector3(actions[0], actions[1], actions[2]);
//        vector3 lower_arm_right_torque = new vector3(actions[3], 0, actions[4]);
//        vector3 hand_right_torque = new vector3(actions[5], 0, actions[6]);

//        vector3 upper_arm_left_torque = new vector3(actions[7], actions[8], actions[9]);
//        vector3 lower_arm_left_torque = new vector3(actions[10], 0, actions[11]);
//        vector3 hand_left_torque = new vector3(actions[12], 0, actions[13]);

//        joints["upper_arm_right"].addforceatposition(upper_arm_right_torque, joints["lower_arm_right"].transform.position);
//        joints["lower_arm_right"].addforceatposition(lower_arm_right_torque, joints["hand_right"].transform.position);
//        joints["hand_right"].addforceatposition(hand_right_torque, limbends["hand_right_end"].position);

//        joints["upper_arm_left"].addforceatposition(upper_arm_left_torque, joints["lower_arm_right"].transform.position);
//        joints["lower_arm_left"].addforceatposition(lower_arm_left_torque, joints["hand_left"].transform.position);
//        joints["hand_left"].addforceatposition(hand_left_torque, limbends["hand_left_end"].position);


//        curdist = vector3.distance(target.position, ball.position);

//        // the reward to evaluate how close the ball to target
//        r_dist = -(curdist - predist) / woodyarea.board_length;
//        addreward(r_dist);

//        // the reward to punish the time cost
//        addreward(-1 / maxstep);

//        //debug.log("the current cumulated reward is: " + getreward());

//        if (woodyarea.board.position.y < 0.3f)
//        {
//            //addreward(-1f);
//            endepisode();
//            debug.log("may not need this check for board dropping on the floor");
//        }
//    }
//    public override void heuristic(in actionbuffers actionsout)
//    {
//        debug.log("heuristic called!");
//        //float[] actions = new float[14];
//        var actions = actionsout.continuousactions;

//        if (input.getkey(keycode.q))
//        {
//            actions[0] = 0.1f;
//        }
//        if (input.getkey(keycode.a))
//        {
//            actions[0] = -0.1f;
//        }
//        if (input.getkey(keycode.w))
//        {
//            actions[1] = 0.1f;
//        }

//        if (input.getkey(keycode.s))
//        {
//            actions[1] = -0.1f;
//        }
//        if (input.getkey(keycode.e))
//        {
//            actions[2] = 0.1f;
//        }
//        if (input.getkey(keycode.d))
//        {
//            actions[2] = -0.1f;
//        }

//        if (input.getkey(keycode.r))
//        {
//            actions[3] = .1f;
//        }
//        if (input.getkey(keycode.f))
//        {
//            actions[3] = .1f;
//        }
//        if (input.getkey(keycode.t))
//        {
//            actions[4] = .1f;
//        }

//        if (input.getkey(keycode.g))
//        {
//            actions[4] = -.1f;
//        }
//        if (input.getkey(keycode.y))
//        {
//            actions[5] = .1f;
//        }
//        if (input.getkey(keycode.h))
//        {
//            actions[5] = -.1f;
//        }

//        if (input.getkey(keycode.u))
//        {
//            actions[6] = .1f;
//        }
//        if (input.getkey(keycode.j))
//        {
//            actions[6] = -.1f;
//        }
//        if (input.getkey(keycode.i))
//        {
//            actions[7] = .1f;
//        }
//        if (input.getkey(keycode.k))
//        {
//            actions[7] = -.1f;
//        }
//        if (input.getkey(keycode.o))
//        {
//            actions[8] = .1f;
//        }
//        if (input.getkey(keycode.l))
//        {
//            actions[8] = -.1f;
//        }
//    }
//    public override void onepisodebegin()
//    {
//        debug.log("onepisodebegin called!");
//        woodyarea.areareset();
//        botbacktooriginal(invertagent);

//    }

//    //-----------------------------------------------------------------------------
//    public void botbacktooriginal(bool invertagent)
//    {
//        //setup the board and ball physics info to make it stay in the air
//        //boardrigid.usegravity = false;
//        boardrigid.iskinematic = true;

//        //ballrigid.usegravity = false;
//        ballrigid.iskinematic = true;

//        connectjoint = false;
//        // remove the current agent in the scene
//        if (agent != null)
//        {
//            //joints["hand_right"].transform.parent = null;
//            //joints["hand_left"].transform.parent = null;
//            destroy(agent);
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

//            agent = instantiate(woodysource, new vector3(x1, y1, z1), quaternion.euler(rx1, ry1, rz1));
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

//            agent = instantiate(woodysource, new vector3(x2, y2, z2), quaternion.euler(rx2, ry2, rz2));
//            agent.transform.parent = woodyarea.box_agent2;
//        }

//        // setup all joints info
//        jointstag.clear();
//        jointstag.add("upper_arm_right");
//        jointstag.add("lower_arm_right");
//        jointstag.add("hand_right");

//        jointstag.add("upper_arm_left");
//        jointstag.add("lower_arm_left");
//        jointstag.add("hand_left");



//        joints.clear();
//        foreach (string tag in jointstag)
//        {
//            findchildobjectwithtag(agent.transform, tag); // store all the joints into the dictionary
//        }

//        limbends.clear();
//        findchildobjectwithtag_transform(agent.transform, "hand_right_end");
//        findchildobjectwithtag_transform(agent.transform, "hand_left_end");

//        // setup the hand movement info
//        reachboardinit(invertagent);
//    }

//    public void findchildobjectwithtag(transform parent, string _tag)
//    {
//        for (int i = 0; i < parent.childcount; i++)
//        {
//            transform child = parent.getchild(i);
//            if (child.tag == _tag)
//            {
//                joints.add(_tag, child.getcomponent<rigidbody>());
//                break;
//            }
//            if (child.childcount > 0)
//            {
//                findchildobjectwithtag(child, _tag);
//            }
//        }
//    }

//    public void findchildobjectwithtag_transform(transform parent, string _tag)
//    {
//        for (int i = 0; i < parent.childcount; i++)
//        {
//            transform child = parent.getchild(i);
//            if (child.tag == _tag)
//            {
//                limbends.add(_tag, child);
//                break;
//            }
//            if (child.childcount > 0)
//            {
//                findchildobjectwithtag_transform(child, _tag);
//            }
//        }
//    }

//    public void reachboardinit(bool invertagent) // get the starting time and init distance from the hand to target joint location
//    {
//        if (invertagent)
//        {
//            ltargetjoint = a1l;
//            rtargetjoint = a1r;
//        }
//        else
//        {
//            ltargetjoint = a2l;
//            rtargetjoint = a2r;
//        }

//        starttime = time.time;

//        rstart = joints["hand_right"].transform;
//        lstart = joints["hand_left"].transform;

//        rjourneylength = vector3.distance(rstart.position, rtargetjoint.position);
//        ljourneylength = vector3.distance(lstart.position, ltargetjoint.position);
//    }

//    public float[] reachfracs() // calculate the fraction where the current hand is, how long the hand has moved along the journey
//    {
//        float[] fracs = new float[2];
//        finishedjourney = (time.time - starttime) * speed;

//        rfracjourney = finishedjourney / rjourneylength;
//        lfracjourney = finishedjourney / ljourneylength;

//        //rfracjourney = -2 * mathf.pow(rfracjourney, 3.0f) + 3 * mathf.pow(rfracjourney, 2.0f);//
//        //lfracjourney = -2 * mathf.pow(lfracjourney, 3.0f) + 3 * mathf.pow(lfracjourney, 2.0f);//

//        fracs[0] = rfracjourney;
//        fracs[1] = lfracjourney;

//        return fracs;
//    }

//    public void reachboard()
//    {

//        joints["hand_right"].transform.position = vector3.slerp(rstart.position, rtargetjoint.position, rfracjourney);
//        joints["hand_left"].transform.position = vector3.slerp(lstart.position, ltargetjoint.position, lfracjourney);

//        joints["hand_right"].transform.rotation = quaternion.slerp(rstart.rotation, rtargetjoint.rotation, rfracjourney);
//        joints["hand_left"].transform.rotation = quaternion.slerp(lstart.rotation, ltargetjoint.rotation, lfracjourney);
//    }

//}
